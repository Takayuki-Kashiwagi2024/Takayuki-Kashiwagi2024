#include <M5Stack.h>
#include <ESP32Servo.h>

// =====================================================
// M5Stack Relay Timer + Motor Control
//
// 【Aボタン】
// ・リレーを即時ON、0.2秒後にOFF
// ・1回目のOFFから0.5秒後に再度ON
// ・2回目のONから0.2秒後にOFF
// ・同時にモーターを正方向へ90度相当動作
// ・正転後、0.3秒停止して逆方向へ戻す
// ・動作後はモーターPWMを停止
// ・A操作回数を加算。30回では停止しない
//
// 【Bボタン】
// ・モーターは動作させない
// ・開始6秒後に1回目のリレーON
// ・リレーON時間は0.5秒
// ・11秒周期
// ・B操作によるリレーONを30回実行して終了
//
// 【Cボタン】
// ・リレーを即時OFF
// ・A/Bの動作とモーター動作を中断
// ・モーターPWMを停止
// ・A/B両方のカウントを0にリセット
// ・READY状態へ戻す
//
// 【時間管理】
// ・リレーはmicros()で非ブロッキング制御
// ・モーターはmillis()で非ブロッキング制御
// ・LCDは原則500msごとに更新
// =====================================================


// =====================================================
// リレー設定
// =====================================================

static const bool RELAY_ACTIVE_HIGH = true;

static const int RELAY_PIN = 21;  // PORT.A G21
static const int RELAY_NC  = 22;  // PORT.A G22


// =====================================================
// Aボタンのリレー時間設定
//
// 時刻はAボタン操作時を基準とする
// 0.0秒 : 1回目ON
// 0.2秒 : 1回目OFF
// 0.7秒 : 2回目ON
// 0.9秒 : 2回目OFF
// =====================================================

static const uint32_t MANUAL_ON_TIME_US = 200000UL;
static const uint32_t MANUAL_OFF_GAP_US = 500000UL;


// =====================================================
// Bボタンのリレー時間設定
// =====================================================

static const uint32_t RELAY_FIRST_DELAY_US = 6000000UL;
static const uint32_t RELAY_ON_TIME_US     = 500000UL;
static const uint32_t RELAY_INTERVAL_US    = 11000000UL;

static const int RELAY_REPEAT_MAX = 30;


// =====================================================
// モーター設定
// 連続回転サーボをG16へ接続
// =====================================================

static const int SERVO_PIN = 16;

static const uint16_t SERVO_MIN_US = 1000;
static const uint16_t SERVO_MAX_US = 2000;

// 実測で停止しやすかった中立値
static const uint16_t NEUTRAL_BASE_US = 1550;

// 必要な場合は、この値を直接変更する
static const int NEUTRAL_TRIM_US = 0;

// 回転速度
static const int DRIVE_OFFSET_US = 200;

// 正方向90度相当の駆動時間
static const uint32_t MOVE_90_FWD_MS = 500;

// 逆方向90度相当の駆動時間
static const uint32_t MOVE_90_BACK_MS = 520;

// 正転後の停止時間
static const uint32_t MOTOR_HOLD_MS = 300;


// =====================================================
// LCD設定
// =====================================================

static const uint32_t LCD_UPDATE_INTERVAL_US = 500000UL;

static const int TITLE_Y  = 10;
static const int GUIDE_Y  = 42;
static const int GUIDE2_Y = 66;
static const int STATUS_Y = 96;
static const int SEC_Y    = 130;
static const int COUNT_Y  = 164;
static const int INFO_Y   = 200;


// =====================================================
// リレー状態
// =====================================================

bool relayOn = false;
bool sequenceRunning = false;

// BボタンのリレーON中かどうか
bool sequencePulseOn = false;

uint32_t relayOffTimeUs = 0;
uint32_t nextRelayOnTimeUs = 0;
uint32_t nextLcdUpdateUs = 0;

// AとBのカウントを分離
uint64_t manualCount = 0;
int sequenceCount = 0;

const char* currentStatus = "READY";


// =====================================================
// Aボタンのリレー動作状態
// =====================================================

enum ManualRelayState
{
    MANUAL_IDLE,
    MANUAL_FIRST_ON,
    MANUAL_OFF_GAP,
    MANUAL_SECOND_ON
};

ManualRelayState manualRelayState = MANUAL_IDLE;

// Aボタン操作時刻
uint32_t manualStartTimeUs = 0;


// =====================================================
// モーター
// =====================================================

Servo servo1;
bool servoAttached = false;

enum MotorState
{
    MOTOR_IDLE,
    MOTOR_FORWARD,
    MOTOR_HOLD,
    MOTOR_BACKWARD
};

MotorState motorState = MOTOR_IDLE;
uint32_t motorStateEndMs = 0;


// =====================================================
// 出力レベル
// =====================================================

inline int relayOnLevel()
{
    return RELAY_ACTIVE_HIGH ? HIGH : LOW;
}

inline int relayOffLevel()
{
    return RELAY_ACTIVE_HIGH ? LOW : HIGH;
}


// =====================================================
// 時刻比較
// micros() / millis() のオーバーフロー対応
// =====================================================

inline bool timeReached(
    uint32_t nowUs,
    uint32_t targetUs
)
{
    return (int32_t)(nowUs - targetUs) >= 0;
}

inline bool timeReachedMs(
    uint32_t nowMs,
    uint32_t targetMs
)
{
    return (int32_t)(nowMs - targetMs) >= 0;
}


// =====================================================
// リレー出力
// =====================================================

void turnRelayOn()
{
    digitalWrite(
        RELAY_PIN,
        relayOnLevel()
    );

    relayOn = true;
}

void turnRelayOff()
{
    digitalWrite(
        RELAY_PIN,
        relayOffLevel()
    );

    relayOn = false;
}


// =====================================================
// LCD表示補助
// =====================================================

void clearLine(int y, int h = 24)
{
    M5.Lcd.fillRect(
        10,
        y,
        310,
        h,
        BLACK
    );
}

void printStatus(const char* text)
{
    clearLine(STATUS_Y);

    M5.Lcd.setCursor(
        10,
        STATUS_Y
    );

    M5.Lcd.setTextColor(
        WHITE,
        BLACK
    );

    M5.Lcd.print(text);
}

void printCount()
{
    clearLine(COUNT_Y);

    M5.Lcd.setCursor(
        10,
        COUNT_Y
    );

    M5.Lcd.setTextColor(
        YELLOW,
        BLACK
    );

    M5.Lcd.printf(
        "A:%llu  B:%d/%d",
        (unsigned long long)manualCount,
        sequenceCount,
        RELAY_REPEAT_MAX
    );
}

void printCountdown(float sec)
{
    clearLine(SEC_Y);

    M5.Lcd.setCursor(
        10,
        SEC_Y
    );

    M5.Lcd.setTextColor(
        GREEN,
        BLACK
    );

    M5.Lcd.printf(
        "Next ON : %.1f s",
        sec
    );
}

void printManualOperation()
{
    clearLine(SEC_Y);

    M5.Lcd.setCursor(
        10,
        SEC_Y
    );

    M5.Lcd.setTextColor(
        MAGENTA,
        BLACK
    );

    switch (motorState)
    {
        case MOTOR_FORWARD:
            M5.Lcd.print("Motor : FORWARD");
            break;

        case MOTOR_HOLD:
            M5.Lcd.print("Motor : HOLD");
            break;

        case MOTOR_BACKWARD:
            M5.Lcd.print("Motor : RETURN");
            break;

        default:
            M5.Lcd.print("Manual Relay");
            break;
    }
}

void printInfo()
{
    clearLine(INFO_Y);

    M5.Lcd.setCursor(
        10,
        INFO_Y
    );

    M5.Lcd.setTextColor(
        CYAN,
        BLACK
    );

    M5.Lcd.printf(
        "B Interval : %.3f s",
        RELAY_INTERVAL_US / 1000000.0f
    );
}


// =====================================================
// LCD更新
// =====================================================

void updateLcd()
{
    uint32_t nowUs = micros();

    if (!timeReached(
            nowUs,
            nextLcdUpdateUs
        ))
    {
        return;
    }

    nextLcdUpdateUs =
        nowUs +
        LCD_UPDATE_INTERVAL_US;

    printStatus(currentStatus);
    printCount();

    // Aボタンの動作中
    if (
        motorState != MOTOR_IDLE ||
        manualRelayState != MANUAL_IDLE
    )
    {
        printManualOperation();
    }
    // Bボタンの次回ON待機中
    else if (
        sequenceRunning &&
        !relayOn &&
        sequenceCount < RELAY_REPEAT_MAX
    )
    {
        int32_t remainUs =
            (int32_t)(
                nextRelayOnTimeUs -
                nowUs
            );

        if (remainUs < 0)
        {
            remainUs = 0;
        }

        printCountdown(
            remainUs / 1000000.0f
        );
    }
    else
    {
        clearLine(SEC_Y);
    }

    printInfo();
}


// =====================================================
// リレー初期化・強制OFF
// =====================================================

void forceRelayOff()
{
    pinMode(
        RELAY_PIN,
        OUTPUT
    );

    pinMode(
        RELAY_NC,
        OUTPUT
    );

    digitalWrite(
        RELAY_PIN,
        relayOffLevel()
    );

    digitalWrite(
        RELAY_NC,
        relayOffLevel()
    );

    relayOn = false;
    sequencePulseOn = false;
    manualRelayState = MANUAL_IDLE;

    delay(2);
}


// =====================================================
// サーボ接続
// =====================================================

void servoAttachIfNeeded()
{
    if (servoAttached)
    {
        return;
    }

    servo1.setPeriodHertz(50);

    servo1.attach(
        SERVO_PIN,
        SERVO_MIN_US,
        SERVO_MAX_US
    );

    servoAttached = true;

    int neutralUs =
        NEUTRAL_BASE_US +
        NEUTRAL_TRIM_US;

    neutralUs = constrain(
        neutralUs,
        (int)SERVO_MIN_US,
        (int)SERVO_MAX_US
    );

    servo1.writeMicroseconds(
        neutralUs
    );
}


// =====================================================
// サーボPWM停止
// =====================================================

void servoDetachIfIdle()
{
    if (!servoAttached)
    {
        return;
    }

    servo1.detach();
    servoAttached = false;
}


// =====================================================
// サーボパルス出力
// =====================================================

void servoWritePulse(int pulseUs)
{
    if (!servoAttached)
    {
        return;
    }

    pulseUs += NEUTRAL_TRIM_US;

    pulseUs = constrain(
        pulseUs,
        (int)SERVO_MIN_US,
        (int)SERVO_MAX_US
    );

    servo1.writeMicroseconds(
        pulseUs
    );
}

void motorStop()
{
    servoWritePulse(
        NEUTRAL_BASE_US
    );
}

void motorDrive(int dir)
{
    int pulseUs =
        (int)NEUTRAL_BASE_US +
        dir * DRIVE_OFFSET_US;

    servoWritePulse(
        pulseUs
    );
}


// =====================================================
// Aボタン用モーター動作開始
// =====================================================

void startMotorSequence()
{
    servoAttachIfNeeded();

    motorDrive(+1);

    motorState = MOTOR_FORWARD;

    motorStateEndMs =
        millis() +
        MOVE_90_FWD_MS;
}


// =====================================================
// モーター動作管理
// =====================================================

void manageMotor()
{
    if (motorState == MOTOR_IDLE)
    {
        return;
    }

    uint32_t nowMs = millis();

    if (!timeReachedMs(
            nowMs,
            motorStateEndMs
        ))
    {
        return;
    }

    switch (motorState)
    {
        case MOTOR_FORWARD:

            motorStop();

            motorState = MOTOR_HOLD;

            motorStateEndMs =
                nowMs +
                MOTOR_HOLD_MS;

            currentStatus =
                "MOTOR HOLD";

            break;

        case MOTOR_HOLD:

            motorDrive(-1);

            motorState =
                MOTOR_BACKWARD;

            motorStateEndMs =
                nowMs +
                MOVE_90_BACK_MS;

            currentStatus =
                "MOTOR RETURN";

            break;

        case MOTOR_BACKWARD:

            motorStop();

            motorState =
                MOTOR_IDLE;

            motorStateEndMs = 0;

            servoDetachIfIdle();

            if (sequenceRunning)
            {
                currentStatus =
                    "RUNNING";
            }
            else if (
                sequenceCount >=
                RELAY_REPEAT_MAX
            )
            {
                currentStatus =
                    "B COMPLETE";
            }
            else
            {
                currentStatus =
                    "READY";
            }

            break;

        default:

            motorStop();

            motorState =
                MOTOR_IDLE;

            motorStateEndMs = 0;

            servoDetachIfIdle();

            break;
    }

    nextLcdUpdateUs = micros();
}


// =====================================================
// モーター動作強制中断
// =====================================================

void abortMotorSequence()
{
    if (servoAttached)
    {
        motorStop();
        delay(5);
    }

    servoDetachIfIdle();

    motorState = MOTOR_IDLE;
    motorStateEndMs = 0;
}


// =====================================================
// Aボタン動作開始
//
// Aの回数はBの30回制限と無関係
// =====================================================

void startManualRelayAndMotor()
{
    // リレー使用中は開始しない
    if (relayOn)
    {
        return;
    }

    // 前回のAリレー動作中は開始しない
    if (manualRelayState != MANUAL_IDLE)
    {
        return;
    }

    // モーター動作中は開始しない
    if (motorState != MOTOR_IDLE)
    {
        return;
    }

    // Bの30回到達状態は判定しない
    // Aは何度でも操作可能

    manualStartTimeUs = micros();

    turnRelayOn();

    manualRelayState =
        MANUAL_FIRST_ON;

    manualCount++;

    currentStatus =
        "MANUAL + MOTOR";

    startMotorSequence();

    nextLcdUpdateUs = micros();
}


// =====================================================
// Aボタンの2回パルス管理
//
// 操作時刻を基準に時刻を決める
// =====================================================

void manageManualRelay()
{
    if (manualRelayState == MANUAL_IDLE)
    {
        return;
    }

    uint32_t nowUs = micros();

    switch (manualRelayState)
    {
        case MANUAL_FIRST_ON:

            // 操作から0.2秒後に1回目OFF
            if (timeReached(
                    nowUs,
                    manualStartTimeUs +
                    MANUAL_ON_TIME_US
                ))
            {
                turnRelayOff();

                manualRelayState =
                    MANUAL_OFF_GAP;

                nextLcdUpdateUs = nowUs;
            }

            break;

        case MANUAL_OFF_GAP:

            // 1回目OFFから0.5秒後に再ON
            if (timeReached(
                    nowUs,
                    manualStartTimeUs +
                    MANUAL_ON_TIME_US +
                    MANUAL_OFF_GAP_US
                ))
            {
                turnRelayOn();

                manualRelayState =
                    MANUAL_SECOND_ON;

                nextLcdUpdateUs = nowUs;
            }

            break;

        case MANUAL_SECOND_ON:

            // 2回目ONから0.2秒後にOFF
            if (timeReached(
                    nowUs,
                    manualStartTimeUs +
                    MANUAL_ON_TIME_US +
                    MANUAL_OFF_GAP_US +
                    MANUAL_ON_TIME_US
                ))
            {
                turnRelayOff();

                manualRelayState =
                    MANUAL_IDLE;

                nextLcdUpdateUs = nowUs;
            }

            break;

        default:

            turnRelayOff();

            manualRelayState =
                MANUAL_IDLE;

            break;
    }
}


// =====================================================
// Bボタンシーケンス用リレーON
// モーターは動作させない
// =====================================================

void startSequenceRelayPulse(
    uint32_t scheduledTimeUs
)
{
    if (relayOn)
    {
        return;
    }

    if (manualRelayState != MANUAL_IDLE)
    {
        return;
    }

    if (motorState != MOTOR_IDLE)
    {
        return;
    }

    if (sequenceCount >= RELAY_REPEAT_MAX)
    {
        return;
    }

    turnRelayOn();

    uint32_t nowUs = micros();

    sequencePulseOn = true;

    relayOffTimeUs =
        nowUs +
        RELAY_ON_TIME_US;

    sequenceCount++;

    currentStatus =
        "RELAY ON";

    // Bの次回ON時刻は予定時刻を基準に計算
    if (sequenceCount < RELAY_REPEAT_MAX)
    {
        nextRelayOnTimeUs =
            scheduledTimeUs +
            RELAY_INTERVAL_US;
    }

    nextLcdUpdateUs = nowUs;
}


// =====================================================
// BボタンのリレーOFF管理
// =====================================================

void manageSequenceRelayOff()
{
    if (!sequencePulseOn)
    {
        return;
    }

    uint32_t nowUs = micros();

    if (!timeReached(
            nowUs,
            relayOffTimeUs
        ))
    {
        return;
    }

    turnRelayOff();

    sequencePulseOn = false;

    if (sequenceCount >= RELAY_REPEAT_MAX)
    {
        sequenceRunning = false;

        currentStatus =
            "B COMPLETE";
    }
    else
    {
        currentStatus =
            "RELAY OFF";
    }

    nextLcdUpdateUs = nowUs;
}


// =====================================================
// Bボタンシーケンス管理
// =====================================================

void manageRelaySchedule()
{
    if (!sequenceRunning)
    {
        return;
    }

    if (relayOn)
    {
        return;
    }

    // Aボタンのリレー動作中はBのONを保留
    if (manualRelayState != MANUAL_IDLE)
    {
        return;
    }

    // Aボタンのモーター動作中も保留
    if (motorState != MOTOR_IDLE)
    {
        return;
    }

    if (sequenceCount >= RELAY_REPEAT_MAX)
    {
        return;
    }

    uint32_t nowUs = micros();

    if (timeReached(
            nowUs,
            nextRelayOnTimeUs
        ))
    {
        startSequenceRelayPulse(
            nextRelayOnTimeUs
        );
    }
}


// =====================================================
// Bボタンによるシーケンス開始
// =====================================================

void startSequence()
{
    if (sequenceRunning)
    {
        return;
    }

    if (relayOn)
    {
        return;
    }

    if (manualRelayState != MANUAL_IDLE)
    {
        return;
    }

    if (motorState != MOTOR_IDLE)
    {
        return;
    }

    // Bだけを30回で制限
    if (sequenceCount >= RELAY_REPEAT_MAX)
    {
        return;
    }

    uint32_t nowUs = micros();

    sequenceRunning = true;

    nextRelayOnTimeUs =
        nowUs +
        RELAY_FIRST_DELAY_US;

    currentStatus =
        "RUNNING";

    nextLcdUpdateUs = nowUs;
}


// =====================================================
// Cボタンによるシステムリセット
// =====================================================

void resetSystem()
{
    // リレーを即時OFF
    turnRelayOff();

    digitalWrite(
        RELAY_NC,
        relayOffLevel()
    );

    // Aボタンのリレー動作を中断
    manualRelayState =
        MANUAL_IDLE;

    manualStartTimeUs = 0;

    // Bボタンのリレー動作を中断
    sequencePulseOn = false;
    sequenceRunning = false;

    relayOffTimeUs = 0;
    nextRelayOnTimeUs = 0;

    // モーター停止、PWM停止
    abortMotorSequence();

    // 両方のカウントをリセット
    manualCount = 0;
    sequenceCount = 0;

    currentStatus =
        "READY";

    nextLcdUpdateUs = micros();

    printStatus(currentStatus);
    printCount();

    clearLine(SEC_Y);

    printInfo();
}


// =====================================================
// ボタン管理
// =====================================================

void manageButtons()
{
    // Cボタンを最優先
    if (M5.BtnC.wasPressed())
    {
        resetSystem();
        return;
    }

    if (M5.BtnA.wasPressed())
    {
        startManualRelayAndMotor();
    }

    if (M5.BtnB.wasPressed())
    {
        startSequence();
    }
}


// =====================================================
// Setup
// =====================================================

void setup()
{
    // M5Stack初期化前にリレーをOFF
    forceRelayOff();

    // PORT.AのG21/G22をGPIOとして使用
    // I2C初期化は無効
    M5.begin(
        true,   // LCD
        true,   // SD
        true,   // Serial
        false   // I2C無効
    );

    // 初期化後も再度リレーをOFF
    forceRelayOff();

    // 起動時はモーターPWMを停止
    servoDetachIfIdle();

    motorState = MOTOR_IDLE;
    motorStateEndMs = 0;

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);

    M5.Lcd.setCursor(
        10,
        TITLE_Y
    );

    M5.Lcd.setTextColor(
        WHITE,
        BLACK
    );

    M5.Lcd.println(
        "Relay + Motor Timer"
    );

    M5.Lcd.setCursor(
        10,
        GUIDE_Y
    );

    M5.Lcd.println(
        "A:Relay+Motor B:Seq"
    );

    M5.Lcd.setCursor(
        10,
        GUIDE2_Y
    );

    M5.Lcd.println(
        "C:Reset / Stop"
    );

    printStatus(currentStatus);
    printCount();

    clearLine(SEC_Y);

    printInfo();

    nextLcdUpdateUs = micros();
}


// =====================================================
// Loop
// =====================================================

void loop()
{
    M5.update();

    // Cボタンを含むボタン操作
    manageButtons();

    // Aボタンの2回パルス管理
    manageManualRelay();

    // Bボタンの0.5秒パルスOFF管理
    manageSequenceRelayOff();

    // Aボタンによるモーター状態管理
    manageMotor();

    // Bボタンによるリレーシーケンス管理
    manageRelaySchedule();

    // LCD定期更新
    updateLcd();
}
