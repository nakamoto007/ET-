#include "challenges.h"

#include "app.h"
#include "ChallengeCommandLogger.h"
#include "DriveBase.h"
#include "DriveController.h"
#include "RobotConfig.h"

#include <cstddef>
#include <cstdint>
#include <kernel.h>

#include <spike/hub/display.h>
#include <spike/hub/imu.h>

namespace {

enum class ChallengeCommandType : uint8_t {
  Forward,
  Backward,
  TurnLeft,
  TurnRight,
};

struct ChallengeCommand {
  ChallengeCommandType type;
  uint8_t count;
};

struct ChallengeRunContext {
  // 同じ起動中に難所を繰り返した時、ログを試走ごとに分離する番号。
  uint32_t run_sequence;
  // 圧縮後のF/B/L/Rを実行した順番。1から始める。
  uint32_t command_sequence;
  // 難所開始時の正面を原点とする90度刻みの目標方位。
  // 実測した旋回終了角では更新せず、命令上のL/Rだけで更新する。
  double grid_heading;
  // 直前の直進方向。-1=B、0=開始直後または旋回後、1=F。
  // BとFが直接隣接する時だけ反転制御を有効にする判定へ使う。
  int last_drive_direction;
};

struct ChallengeCommandSnapshot {
  SYSTIM time_us;
  bool time_ready;
  bool motors_ready;
  int32_t left_count;
  int32_t right_count;
  double heading;
};

uint32_t challenge_run_sequence = 0;

// 既定経路を実行単位へ事前圧縮する。実機ではこの表を順に読むだけなので、
// 長いF/B/L/R文字列の長さや解析処理が走行開始時の挙動へ影響しない。
const ChallengeCommand DEFAULT_CHALLENGE_COMMANDS[] = {
  {ChallengeCommandType::Forward, 6},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 8},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 1},
  {ChallengeCommandType::Forward, 1},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 7},
  {ChallengeCommandType::Forward, 3},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Forward, 2},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 4},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Forward, 2},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 7},
  {ChallengeCommandType::Forward, 3},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Forward, 2},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 4},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Forward, 2},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 7},
  {ChallengeCommandType::Forward, 3},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Forward, 2},
  {ChallengeCommandType::TurnRight, 1},
  {ChallengeCommandType::Backward, 6},
};

bool isIgnoredStep(char step)
{
  return step == ' ' || step == '\n' || step == '\r' || step == '\t';
}

int countSameSteps(const char *steps, int start_index, char target_step)
{
  int count = 0;
  while (steps[start_index + count] == target_step) {
    ++count;
  }
  return count;
}

// 方位格子をIMUのheading表現と同じ-180..180度へ収める。
// 13回の右旋回を行っても値を連続加算で大きくせず、ログを読みやすく保つ。
double normalizeGridHeading(double heading)
{
  while (heading > 180.0) {
    heading -= 360.0;
  }
  while (heading < -180.0) {
    heading += 360.0;
  }
  return heading;
}

// 命令境界の時刻、左右エンコーダ、方位を同じ場所で取得する。
// モーター取得に失敗しても方位と結果は残し、cmok=0でログから判別できる。
ChallengeCommandSnapshot captureCommandSnapshot(void)
{
  ChallengeCommandSnapshot snapshot = {};
  snapshot.time_ready = get_tim(&snapshot.time_us) == E_OK;
  snapshot.heading = hub_imu_get_heading();

  etrobo_app::DriveMotors motors = {};
  snapshot.motors_ready = etrobo_app::getDriveMotors(&motors);
  if (snapshot.motors_ready) {
    snapshot.left_count = pup_motor_get_count(motors.left);
    snapshot.right_count = pup_motor_get_count(motors.right);
  }
  return snapshot;
}

char commandStep(ChallengeCommandType type)
{
  switch (type) {
  case ChallengeCommandType::Forward:
    return 'F';
  case ChallengeCommandType::Backward:
    return 'B';
  case ChallengeCommandType::TurnLeft:
    return 'L';
  case ChallengeCommandType::TurnRight:
    return 'R';
  default:
    return '?';
  }
}

// 直進は符号付きmm、旋回は符号付きdegへ分け、CSVで単位を混同しないようにする。
int commandTargetDistanceMm(ChallengeCommandType type, int count)
{
  if (type == ChallengeCommandType::Forward) {
    return count * etrobo_app::CHALLENGE_STEP_FORWARD_DISTANCE_MM;
  }
  if (type == ChallengeCommandType::Backward) {
    return -count * etrobo_app::CHALLENGE_STEP_BACKWARD_DISTANCE_MM;
  }
  return 0;
}

int commandTargetDegrees(ChallengeCommandType type, int count)
{
  if (type == ChallengeCommandType::TurnLeft) {
    return -90 * count;
  }
  if (type == ChallengeCommandType::TurnRight) {
    return 90 * count;
  }
  return 0;
}

// F/Bでは現在の格子方位、L/Rでは旋回後の格子方位をログ上の目標とする。
double commandTargetHeading(ChallengeCommandType type,
                            int count,
                            double current_grid_heading)
{
  return normalizeGridHeading(
    current_grid_heading + commandTargetDegrees(type, count));
}

// 走行中にBluetooth送信を行わず、開始・終了差分を非同期ログキューへ積む。
void enqueueCommandLog(const ChallengeRunContext &context,
                       ChallengeCommandType type,
                       int count,
                       bool backward_to_forward_control,
                       double target_heading,
                       const ChallengeCommandSnapshot &start,
                       const ChallengeCommandSnapshot &end,
                       challenge_run_result_t result)
{
  etrobo_app::ChallengeCommandLogEntry entry = {};
  entry.run_sequence = context.run_sequence;
  entry.command_sequence = context.command_sequence;
  entry.command = commandStep(type);
  entry.count = count;
  entry.target_distance_mm = commandTargetDistanceMm(type, count);
  entry.target_degrees = commandTargetDegrees(type, count);
  entry.backward_to_forward_control = backward_to_forward_control;
  entry.backward_compensation_mm = backward_to_forward_control ?
    etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_BACKWARD_COMPENSATION_MM : 0;
  entry.forward_compensation_mm = backward_to_forward_control ?
    etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_FORWARD_COMPENSATION_MM : 0;
  entry.start_time_ms = start.time_ready ?
    static_cast<uint32_t>(start.time_us / 1000) : 0;
  entry.duration_ms = start.time_ready && end.time_ready &&
                      end.time_us >= start.time_us ?
    static_cast<uint32_t>((end.time_us - start.time_us) / 1000) : 0;
  entry.motors_ready = start.motors_ready && end.motors_ready;
  entry.left_start_count = start.left_count;
  entry.right_start_count = start.right_count;
  entry.left_end_count = end.left_count;
  entry.right_end_count = end.right_count;
  entry.left_delta_count = end.left_count - start.left_count;
  entry.right_delta_count = end.right_count - start.right_count;
  entry.start_heading = static_cast<float>(start.heading);
  entry.end_heading = static_cast<float>(end.heading);
  entry.heading_delta = static_cast<float>(
    normalizeGridHeading(end.heading - start.heading));
  entry.target_heading = static_cast<float>(target_heading);
  entry.result = static_cast<int>(result);
  (void)etrobo_app::challengeCommandLoggerEnqueue(entry);
}

// 外部から渡された1文字を、内部で使用する型付きコマンドへ変換する。
// falseはF/B/L/R以外を表し、モーターを動かす前の入力検証にも使用する。
bool commandTypeForStep(char step, ChallengeCommandType *type)
{
  if (type == nullptr) {
    return false;
  }

  switch (step) {
  case 'F':
    *type = ChallengeCommandType::Forward;
    return true;
  case 'B':
    *type = ChallengeCommandType::Backward;
    return true;
  case 'L':
    *type = ChallengeCommandType::TurnLeft;
    return true;
  case 'R':
    *type = ChallengeCommandType::TurnRight;
    return true;
  default:
    return false;
  }
}

// 文字列全体を走行前に検証する。途中に不正文字があった場合、
// そこまで走ってから停止するのではなく、開始前にエラーを返す。
challenge_run_result_t validateStepString(const char *steps)
{
  bool has_command = false;
  for (int index = 0; steps[index] != '\0'; ++index) {
    const char step = steps[index];
    if (isIgnoredStep(step)) {
      continue;
    }

    ChallengeCommandType type = ChallengeCommandType::Forward;
    if (!commandTypeForStep(step, &type)) {
      return CHALLENGE_RUN_RESULT_INVALID_STEP;
    }
    has_command = true;
  }

  return has_command ? CHALLENGE_RUN_RESULT_OK :
                       CHALLENGE_RUN_RESULT_EMPTY_STEPS;
}

// 開始操作で機体がわずかに回された場合でも、その向きを最初の直進方向とする。
// IMU自体はリセットせず、直進PID目標と90度方位格子の原点を現在値へ合わせる。
void prepareChallengeRun(ChallengeRunContext *context)
{
  stop_motors();
  dly_tsk(etrobo_app::CHALLENGE_START_SETTLE_TIME_US);
  reset_straight_pid_heading();
  context->run_sequence = ++challenge_run_sequence;
  context->command_sequence = 0;
  context->grid_heading = hub_imu_get_heading();
  context->last_drive_direction = 0;
}

challenge_run_result_t runForwardSteps(int count, bool follows_backward)
{
  if (count <= 0) {
    return CHALLENGE_RUN_RESULT_OK;
  }

  hub_display_char('F');
  const int distance_mm =
    count * etrobo_app::CHALLENGE_STEP_FORWARD_DISTANCE_MM;
  drive_result_t result = DRIVE_RESULT_OK;
  const bool use_direction_change_control =
    follows_backward &&
    etrobo_app::ENABLE_CHALLENGE_BACKWARD_TO_FORWARD_CONTROL;
  if (use_direction_change_control) {
    // B→F反転で失われる固定距離を、低速の追加後退でB側へ一度だけ補う。
    // Bコマンドの個数に比例させないことで、通常の後退距離校正と切り分ける。
    const int backward_compensation_mm =
      etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_BACKWARD_COMPENSATION_MM;
    if (backward_compensation_mm > 0) {
      result = drive_straight_mm(
        etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_COMPENSATION_SPEED_DEG_S,
        -backward_compensation_mm);
      if (result != DRIVE_RESULT_OK) {
        stop_motors();
        return CHALLENGE_RUN_RESULT_DRIVE_FAILED;
      }
    }

    // 補正後の後退位置をholdしてから低速で前進し、正逆転時の揺り戻しと滑りを抑える。
    hold_motors();
    dly_tsk(etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_HOLD_TIME_US);

    // 前進側のバックラッシュで失われる固定距離も、反転1回につき一度だけ足す。
    const int forward_distance_mm =
      distance_mm +
      etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_FORWARD_COMPENSATION_MM;

    // 短いFでも停止区間を残すため、加速距離は全距離の半分以下に制限する。
    const int half_distance_mm = forward_distance_mm / 2;
    const int configured_accel_distance_mm =
      etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_ACCEL_DISTANCE_MM;
    const int accel_distance_mm =
      configured_accel_distance_mm < half_distance_mm ?
      configured_accel_distance_mm : half_distance_mm;
    if (accel_distance_mm > 0 &&
        etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_START_SPEED_DEG_S <
          etrobo_app::CHALLENGE_STEP_FORWARD_SPEED_DEG_S) {
      result = speed_up(
        etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_START_SPEED_DEG_S,
        etrobo_app::CHALLENGE_STEP_FORWARD_SPEED_DEG_S,
        accel_distance_mm);
      if (result == DRIVE_RESULT_OK) {
        result = drive_straight_mm(
          etrobo_app::CHALLENGE_STEP_FORWARD_SPEED_DEG_S,
          forward_distance_mm - accel_distance_mm);
      }
    } else {
      result = drive_straight_mm(
        etrobo_app::CHALLENGE_BACKWARD_TO_FORWARD_START_SPEED_DEG_S,
        forward_distance_mm);
    }
  } else {
    result = drive_straight_mm(
      etrobo_app::CHALLENGE_STEP_FORWARD_SPEED_DEG_S,
      distance_mm);
  }
  if (result != DRIVE_RESULT_OK) {
    stop_motors();
    return CHALLENGE_RUN_RESULT_DRIVE_FAILED;
  }
  return CHALLENGE_RUN_RESULT_OK;
}

challenge_run_result_t runBackwardSteps(int count)
{
  if (count <= 0) {
    return CHALLENGE_RUN_RESULT_OK;
  }

  hub_display_char('B');
  // 後退は前進と異なる実機誤差を持つため、専用の距離・速度設定を使う。
  // F側の調整値を変えず、Bだけ実測結果に合わせて補正できる。
  const int distance_mm =
    -count * etrobo_app::CHALLENGE_STEP_BACKWARD_DISTANCE_MM;
  const drive_result_t result =
    drive_straight_mm(etrobo_app::CHALLENGE_STEP_BACKWARD_SPEED_DEG_S,
                      distance_mm);
  if (result != DRIVE_RESULT_OK) {
    stop_motors();
    return CHALLENGE_RUN_RESULT_DRIVE_FAILED;
  }
  return CHALLENGE_RUN_RESULT_OK;
}

challenge_run_result_t runTurnSteps(char step,
                                    int count,
                                    ChallengeRunContext *context)
{
  if (count <= 0) {
    return CHALLENGE_RUN_RESULT_OK;
  }

  // 現在の実測方位へ90度を足すのではなく、前回の格子目標から次の格子を作る。
  // これにより、各旋回や直進で残った角度誤差が次の目標へ累積しない。
  const double heading_delta = (step == 'L' ? -90.0 : 90.0) * count;
  const double target_heading =
    normalizeGridHeading(context->grid_heading + heading_delta);
  hub_display_char(step);
  const int command_degrees = (step == 'L' ? -90 : 90) * count;
  // ジャイロ信頼性の切り分け中は、左右エンコーダを終了条件にして直接旋回する。
  // falseへ戻せば、従来の90度絶対方位格子による旋回をそのまま比較できる。
  const int result = etrobo_app::USE_ENCODER_PRIMARY_CHALLENGE_TURN ?
    turn_by_encoder(etrobo_app::CHALLENGE_STEP_TURN_SPEED_DEG_S,
                    command_degrees) :
    turn_to_heading(etrobo_app::CHALLENGE_STEP_TURN_SPEED_DEG_S,
                    target_heading);
  if (result != TURN_RESULT_OK) {
    stop_motors();
    return CHALLENGE_RUN_RESULT_TURN_FAILED;
  }
  context->grid_heading = target_heading;
  return CHALLENGE_RUN_RESULT_OK;
}

// 圧縮コマンドと文字列APIで共通利用する実行窓口。
// countを距離または90度単位へ展開し、既存の走行制御へ渡す。
challenge_run_result_t runCommand(ChallengeCommandType type,
                                  int count,
                                  ChallengeRunContext *context)
{
  ++context->command_sequence;
  const bool follows_backward = context->last_drive_direction < 0;
  const bool backward_to_forward_control =
    type == ChallengeCommandType::Forward &&
    follows_backward &&
    etrobo_app::ENABLE_CHALLENGE_BACKWARD_TO_FORWARD_CONTROL;
  const double target_heading =
    commandTargetHeading(type, count, context->grid_heading);
  const ChallengeCommandSnapshot start = captureCommandSnapshot();
  challenge_run_result_t result = CHALLENGE_RUN_RESULT_INVALID_STEP;

  switch (type) {
  case ChallengeCommandType::Forward: {
    result = runForwardSteps(count, follows_backward);
    if (result == CHALLENGE_RUN_RESULT_OK) {
      context->last_drive_direction = 1;
    }
    break;
  }
  case ChallengeCommandType::Backward: {
    result = runBackwardSteps(count);
    if (result == CHALLENGE_RUN_RESULT_OK) {
      context->last_drive_direction = -1;
    }
    break;
  }
  case ChallengeCommandType::TurnLeft: {
    result = runTurnSteps('L', count, context);
    if (result == CHALLENGE_RUN_RESULT_OK) {
      context->last_drive_direction = 0;
    }
    break;
  }
  case ChallengeCommandType::TurnRight: {
    result = runTurnSteps('R', count, context);
    if (result == CHALLENGE_RUN_RESULT_OK) {
      context->last_drive_direction = 0;
    }
    break;
  }
  default:
    stop_motors();
    result = CHALLENGE_RUN_RESULT_INVALID_STEP;
    break;
  }

  const ChallengeCommandSnapshot end = captureCommandSnapshot();
  enqueueCommandLog(*context, type, count,
                    backward_to_forward_control, target_heading,
                    start, end, result);
  return result;
}

// 既定経路は文字列へ展開せず、事前圧縮した31コマンドを直接実行する。
challenge_run_result_t runDefaultCommands(void)
{
  ChallengeRunContext context = {};
  prepareChallengeRun(&context);
  const size_t command_count =
    sizeof(DEFAULT_CHALLENGE_COMMANDS) / sizeof(DEFAULT_CHALLENGE_COMMANDS[0]);
  for (size_t index = 0; index < command_count; ++index) {
    const ChallengeCommand &command = DEFAULT_CHALLENGE_COMMANDS[index];
    if (command.count == 0) {
      stop_motors();
      return CHALLENGE_RUN_RESULT_INVALID_STEP;
    }

    const challenge_run_result_t result =
      runCommand(command.type, static_cast<int>(command.count), &context);
    if (result != CHALLENGE_RUN_RESULT_OK) {
      return result;
    }
  }

  stop_motors();
  return CHALLENGE_RUN_RESULT_OK;
}

}  // namespace

challenge_run_result_t challenges_run_steps(const char *steps)
{
  if (steps == nullptr) {
    return CHALLENGE_RUN_RESULT_NULL_STEPS;
  }

  const challenge_run_result_t validation_result = validateStepString(steps);
  if (validation_result != CHALLENGE_RUN_RESULT_OK) {
    stop_motors();
    return validation_result;
  }

  ChallengeRunContext context = {};
  prepareChallengeRun(&context);
  for (int index = 0; steps[index] != '\0';) {
    const char step = steps[index];
    if (isIgnoredStep(step)) {
      ++index;
      continue;
    }

    ChallengeCommandType type = ChallengeCommandType::Forward;
    if (!commandTypeForStep(step, &type)) {
      stop_motors();
      return CHALLENGE_RUN_RESULT_INVALID_STEP;
    }

    const int count = countSameSteps(steps, index, step);
    const challenge_run_result_t result = runCommand(type, count, &context);
    if (result != CHALLENGE_RUN_RESULT_OK) {
      return result;
    }
    index += count;
  }

  stop_motors();
  return CHALLENGE_RUN_RESULT_OK;
}

challenge_run_result_t challenges_run_default_steps(void)
{
  return runDefaultCommands();
}
