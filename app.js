// Embedded AI Copilot Application Logic

class EmbeddedAICopilot {
    constructor() {
        this.messages = [];
        this.currentTheme = 'light';
        this.isTyping = false;
        
        // Demo responses to simulate AI behavior
        this.demoResponses = [
            `Great question! Here's how to configure SPI DMA for sensor streaming:

\`\`\`c
#include "stm32f4xx_hal.h"

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

uint8_t sensor_buffer[128];
volatile uint8_t dma_complete = 0;

HAL_StatusTypeDef spi_dma_init(void)
{
    // Configure SPI
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

void start_sensor_streaming(void)
{
    HAL_SPI_Receive_DMA(&hspi1, sensor_buffer, 128);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        dma_complete = 1;
        // Process sensor data here
        process_sensor_data(sensor_buffer, 128);
    }
}
\`\`\`

Key considerations:
• Configure DMA channels properly for optimal performance
• Use circular buffer for continuous streaming
• Implement proper error handling and timeouts
• Consider power management during idle periods`,

            `For ECU task scheduling, here's an optimized approach using priority-based scheduling:

\`\`\`c
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

// Task priorities (higher number = higher priority)
#define SAFETY_TASK_PRIORITY      (5)
#define SENSOR_TASK_PRIORITY      (4)
#define CONTROL_TASK_PRIORITY     (3)
#define COMM_TASK_PRIORITY        (2)
#define DIAG_TASK_PRIORITY        (1)

// Task periods in milliseconds
#define SAFETY_PERIOD_MS          (10)
#define SENSOR_PERIOD_MS          (20)
#define CONTROL_PERIOD_MS         (50)
#define COMM_PERIOD_MS            (100)

typedef struct {
    TaskHandle_t handle;
    TickType_t period;
    TickType_t last_wake_time;
} ecu_task_t;

static ecu_task_t ecu_tasks[5];

void safety_monitoring_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for(;;) {
        // Critical safety checks
        check_watchdog_status();
        verify_sensor_plausibility();
        monitor_voltage_levels();
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}
\`\`\`

Best practices:
• Use rate-monotonic scheduling for predictable timing
• Implement stack overflow protection
• Monitor CPU utilization and adjust priorities accordingly
• Use inter-task communication queues for data sharing`,

            `Here's a complete AUTOSAR BSW implementation for CAN communication:

\`\`\`c
// CanIf.c - CAN Interface Module
#include "CanIf.h"
#include "Can.h"
#include "PduR.h"

#define CANIF_MAX_CONTROLLERS  (2)
#define CANIF_MAX_CHANNELS     (16)

typedef struct {
    Can_StateTransitionType state;
    uint8_t controller_mode;
    boolean wakeup_validation;
} CanIf_ControllerState_t;

static CanIf_ControllerState_t canif_controller_state[CANIF_MAX_CONTROLLERS];

Std_ReturnType CanIf_Init(const CanIf_ConfigType* ConfigPtr)
{
    uint8_t controller_idx;
    
    for (controller_idx = 0; controller_idx < CANIF_MAX_CONTROLLERS; controller_idx++) {
        canif_controller_state[controller_idx].state = CAN_T_STOP;
        canif_controller_state[controller_idx].controller_mode = CANIF_CS_UNINIT;
        canif_controller_state[controller_idx].wakeup_validation = FALSE;
    }
    
    // Initialize CAN driver
    Can_Init(&Can_Config);
    
    return E_OK;
}

Std_ReturnType CanIf_Transmit(PduIdType TxPduId, const PduInfoType* PduInfoPtr)
{
    Can_PduType can_pdu;
    Can_HwHandleType hth;
    
    // Map PDU ID to hardware transmit handle
    hth = CanIf_GetHthFromPduId(TxPduId);
    
    can_pdu.id = CanIf_GetCanIdFromPduId(TxPduId);
    can_pdu.length = PduInfoPtr->SduLength;
    can_pdu.sdu = PduInfoPtr->SduDataPtr;
    can_pdu.swPduHandle = TxPduId;
    
    return Can_Write(hth, &can_pdu);
}
\`\`\`

Implementation notes:
• Follow AUTOSAR methodology for layered architecture
• Implement proper error handling and DET reporting
• Use configuration tools for parameter generation
• Ensure timing requirements are met for real-time constraints`,

            `For ISO 26262 compliance, here's a comprehensive safety implementation:

\`\`\`cpp
// SafetyManager.cpp - ASIL D compliant safety manager
#include "SafetyManager.h"
#include "DiagnosticManager.h"

class SafetyManager {
private:
    static constexpr uint32_t SAFETY_HEARTBEAT_TIMEOUT_MS = 100;
    static constexpr uint32_t MAX_FAULT_COUNT = 3;
    
    struct SafetyContext {
        uint32_t heartbeat_counter;
        uint32_t fault_counter;
        SafetyState current_state;
        SafetyState target_state;
        bool degraded_mode_active;
    };
    
    SafetyContext safety_ctx;
    
public:
    SafetyResult initialize() {
        safety_ctx.heartbeat_counter = 0;
        safety_ctx.fault_counter = 0;
        safety_ctx.current_state = SAFETY_STATE_INIT;
        safety_ctx.target_state = SAFETY_STATE_NORMAL;
        safety_ctx.degraded_mode_active = false;
        
        // Initialize hardware safety features
        enable_watchdog();
        configure_safety_interrupts();
        
        return SAFETY_OK;
    }
    
    SafetyResult monitor_safety_functions() {
        // 1. Check critical sensor plausibility
        if (!check_sensor_plausibility()) {
            report_safety_fault(FAULT_SENSOR_PLAUSIBILITY);
            return SAFETY_FAULT_DETECTED;
        }
        
        // 2. Verify actuator responses
        if (!verify_actuator_feedback()) {
            report_safety_fault(FAULT_ACTUATOR_RESPONSE);
            return SAFETY_FAULT_DETECTED;
        }
        
        // 3. Monitor communication integrity
        if (!check_communication_integrity()) {
            report_safety_fault(FAULT_COMMUNICATION);
            return SAFETY_FAULT_DETECTED;
        }
        
        return SAFETY_OK;
    }
    
    void handle_safety_fault(SafetyFaultType fault_type) {
        safety_ctx.fault_counter++;
        
        if (safety_ctx.fault_counter >= MAX_FAULT_COUNT) {
            transition_to_safe_state();
        } else {
            activate_degraded_mode();
        }
    }
};
\`\`\`

Safety implementation checklist:
• Implement dual-channel monitoring for ASIL D requirements
• Use diverse software and hardware for fault detection
• Implement graceful degradation mechanisms
• Ensure fail-safe behavior in all fault conditions
• Document all safety-relevant design decisions`,

            `Here's an advanced sensor fusion algorithm for automotive applications:

\`\`\`c
#include "sensor_fusion.h"
#include "kalman_filter.h"

#define MAX_SENSORS 8
#define STATE_VECTOR_SIZE 6  // [x, y, vx, vy, ax, ay]

typedef struct {
    float position[2];      // x, y coordinates
    float velocity[2];      // vx, vy
    float acceleration[2];  // ax, ay
    float covariance[STATE_VECTOR_SIZE][STATE_VECTOR_SIZE];
    uint32_t timestamp;
} vehicle_state_t;

typedef struct {
    sensor_type_t type;
    float data[4];
    float noise_variance;
    uint32_t timestamp;
    bool valid;
} sensor_data_t;

static vehicle_state_t estimated_state;
static sensor_data_t sensor_inputs[MAX_SENSORS];
static kalman_filter_t kf;

void sensor_fusion_init(void)
{
    // Initialize Kalman filter
    kalman_filter_init(&kf, STATE_VECTOR_SIZE);
    
    // Set initial state covariance
    float initial_covariance[STATE_VECTOR_SIZE] = {
        10.0f, 10.0f,    // position uncertainty
        5.0f, 5.0f,      // velocity uncertainty
        2.0f, 2.0f       // acceleration uncertainty
    };
    
    kalman_set_initial_covariance(&kf, initial_covariance);
}

void update_sensor_fusion(uint32_t current_time)
{
    // Prediction step
    kalman_predict(&kf, current_time);
    
    // Update with each valid sensor
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (sensor_inputs[i].valid) {
            switch (sensor_inputs[i].type) {
                case SENSOR_GPS:
                    update_with_gps(&sensor_inputs[i]);
                    break;
                case SENSOR_IMU:
                    update_with_imu(&sensor_inputs[i]);
                    break;
                case SENSOR_WHEEL_SPEED:
                    update_with_wheel_speed(&sensor_inputs[i]);
                    break;
                case SENSOR_LIDAR:
                    update_with_lidar(&sensor_inputs[i]);
                    break;
            }
        }
    }
    
    // Extract final state estimate
    kalman_get_state(&kf, &estimated_state);
}
\`\`\`

Key features:
• Multi-sensor data fusion using Extended Kalman Filter
• Handles sensor failures gracefully
• Provides uncertainty estimates for downstream systems
• Optimized for real-time performance`,

            `Here's a robust ECU diagnostic implementation with UDS protocol support:

\`\`\`c
// UDS_Diagnostic.c - Unified Diagnostic Services
#include "UDS_Diagnostic.h"
#include "Flash_Manager.h"
#include "DTC_Manager.h"

#define UDS_BUFFER_SIZE 4095
#define MAX_DTC_COUNT 256

typedef struct {
    uint32_t dtc_code;
    uint8_t status_byte;
    uint8_t severity;
    uint32_t occurrence_count;
    uint32_t first_occurrence_time;
    uint32_t last_occurrence_time;
} dtc_record_t;

static uint8_t uds_buffer[UDS_BUFFER_SIZE];
static dtc_record_t dtc_memory[MAX_DTC_COUNT];
static uint16_t active_dtc_count = 0;

// Service 0x22 - Read Data By Identifier
Std_ReturnType UDS_ReadDataByIdentifier(uint16_t data_id, uint8_t* response_data, uint16_t* response_length)
{
    switch (data_id) {
        case 0xF186:  // Active Diagnostic Session
            response_data[0] = get_current_diagnostic_session();
            *response_length = 1;
            break;
            
        case 0xF190:  // VIN Data Identifier
            get_vehicle_identification_number(response_data);
            *response_length = 17;
            break;
            
        case 0xF195:  // ECU Manufacturing Date
            get_ecu_manufacturing_date(response_data);
            *response_length = 4;
            break;
            
        case 0xF1A0:  // ECU Serial Number
            get_ecu_serial_number(response_data);
            *response_length = 16;
            break;
            
        default:
            return E_NOT_OK;
    }
    
    return E_OK;
}

// Service 0x19 - Read DTC Information
Std_ReturnType UDS_ReadDTCInformation(uint8_t sub_function, uint8_t* response_data, uint16_t* response_length)
{
    uint16_t dtc_count = 0;
    uint16_t response_index = 0;
    
    switch (sub_function) {
        case 0x01:  // Report Number of DTC by Status Mask
            dtc_count = count_dtc_by_status_mask(0xFF);
            response_data[0] = DTC_FORMAT_IDENTIFIER;
            response_data[1] = (dtc_count >> 8) & 0xFF;
            response_data[2] = dtc_count & 0xFF;
            *response_length = 3;
            break;
            
        case 0x02:  // Report DTC by Status Mask
            response_data[response_index++] = DTC_FORMAT_IDENTIFIER;
            
            for (uint16_t i = 0; i < MAX_DTC_COUNT; i++) {
                if (dtc_memory[i].status_byte != 0) {
                    // Add DTC to response
                    response_data[response_index++] = (dtc_memory[i].dtc_code >> 16) & 0xFF;
                    response_data[response_index++] = (dtc_memory[i].dtc_code >> 8) & 0xFF;
                    response_data[response_index++] = dtc_memory[i].dtc_code & 0xFF;
                    response_data[response_index++] = dtc_memory[i].status_byte;
                }
            }
            *response_length = response_index;
            break;
            
        default:
            return E_NOT_OK;
    }
    
    return E_OK;
}
\`\`\`

Diagnostic features:
• Full UDS protocol compliance
• Persistent DTC storage in EEPROM
• Freeze frame data capture
• Security access implementation
• Bootloader communication support`,

            `Here's an efficient power management system for automotive ECUs:

\`\`\`c
// PowerManager.c - Advanced ECU Power Management
#include "PowerManager.h"
#include "ClockManager.h"
#include "WakeupManager.h"

typedef enum {
    POWER_MODE_NORMAL = 0,
    POWER_MODE_ECONOMY,
    POWER_MODE_SLEEP,
    POWER_MODE_DEEP_SLEEP,
    POWER_MODE_SHUTDOWN
} power_mode_t;

typedef struct {
    power_mode_t current_mode;
    power_mode_t target_mode;
    uint32_t mode_transition_time;
    uint32_t last_activity_time;
    uint32_t sleep_timeout_ms;
    bool wakeup_pending;
} power_context_t;

static power_context_t power_ctx;

void power_manager_init(void)
{
    power_ctx.current_mode = POWER_MODE_NORMAL;
    power_ctx.target_mode = POWER_MODE_NORMAL;
    power_ctx.sleep_timeout_ms = 30000; // 30 seconds
    power_ctx.wakeup_pending = false;
    
    // Configure wakeup sources
    configure_can_wakeup();
    configure_gpio_wakeup();
    configure_timer_wakeup();
}

void power_manager_update(void)
{
    uint32_t current_time = get_system_time_ms();
    uint32_t idle_time = current_time - power_ctx.last_activity_time;
    
    // Check for mode transition conditions
    if (idle_time > power_ctx.sleep_timeout_ms && 
        power_ctx.current_mode == POWER_MODE_NORMAL) {
        
        request_power_mode_transition(POWER_MODE_ECONOMY);
    }
    
    // Handle mode transitions
    if (power_ctx.current_mode != power_ctx.target_mode) {
        handle_power_mode_transition();
    }
    
    // Monitor power consumption
    monitor_power_consumption();
}

static void handle_power_mode_transition(void)
{
    switch (power_ctx.target_mode) {
        case POWER_MODE_ECONOMY:
            // Reduce CPU frequency
            set_cpu_frequency(FREQ_LOW);
            
            // Disable non-essential peripherals
            disable_non_critical_peripherals();
            
            // Reduce CAN bus activity
            set_can_bus_mode(CAN_MODE_LISTEN_ONLY);
            break;
            
        case POWER_MODE_SLEEP:
            // Save critical data to RAM
            save_critical_context();
            
            // Configure wakeup sources
            enable_wakeup_sources();
            
            // Enter CPU sleep mode
            enter_cpu_sleep_mode();
            break;
            
        case POWER_MODE_DEEP_SLEEP:
            // Save state to non-volatile memory
            save_state_to_eeprom();
            
            // Power down most peripherals
            shutdown_peripherals();
            
            // Enter deep sleep
            enter_deep_sleep_mode();
            break;
    }
    
    power_ctx.current_mode = power_ctx.target_mode;
}

void power_manager_wakeup_handler(wakeup_source_t source)
{
    power_ctx.wakeup_pending = true;
    power_ctx.last_activity_time = get_system_time_ms();
    
    // Restore normal operation
    if (power_ctx.current_mode != POWER_MODE_NORMAL) {
        request_power_mode_transition(POWER_MODE_NORMAL);
        restore_system_context();
    }
}
\`\`\`

Power management features:
• Intelligent sleep mode transitions
• Configurable wakeup sources
• Power consumption monitoring
• Context save/restore mechanisms
• Network management integration`,

            `Here's a comprehensive functional safety implementation for brake-by-wire systems:

\`\`\`c
// BrakeByWire_Safety.c - ASIL D Brake System
#include "BrakeByWire_Safety.h"
#include "SafetyLibrary.h"

#define BRAKE_CHANNELS 4
#define SAFETY_MARGIN_PERCENT 15
#define MAX_DECELERATION_G 1.2f

typedef struct {
    float pressure_setpoint;
    float pressure_actual;
    float pressure_error;
    bool channel_fault;
    uint32_t fault_count;
    safety_state_t safety_state;
} brake_channel_t;

typedef struct {
    brake_channel_t channels[BRAKE_CHANNELS];
    float vehicle_speed;
    float deceleration_request;
    float actual_deceleration;
    bool emergency_brake_active;
    safety_integrity_level_t asil_level;
} brake_system_t;

static brake_system_t brake_system;

// Dual-channel safety monitoring
static float primary_brake_calculation(float brake_request, float vehicle_speed);
static float secondary_brake_calculation(float brake_request, float vehicle_speed);

Std_ReturnType brake_safety_init(void)
{
    brake_system.asil_level = ASIL_D;
    brake_system.emergency_brake_active = false;
    
    for (int i = 0; i < BRAKE_CHANNELS; i++) {
        brake_system.channels[i].safety_state = SAFETY_STATE_NORMAL;
        brake_system.channels[i].fault_count = 0;
        brake_system.channels[i].channel_fault = false;
    }
    
    // Initialize safety-critical timers
    init_safety_timers();
    
    // Configure diverse hardware monitoring
    configure_pressure_sensors();
    configure_brake_actuators();
    
    return E_OK;
}

Std_ReturnType brake_safety_monitor(void)
{
    float primary_result, secondary_result;
    float deviation_percent;
    
    // Dual-channel calculation
    primary_result = primary_brake_calculation(
        brake_system.deceleration_request, 
        brake_system.vehicle_speed
    );
    
    secondary_result = secondary_brake_calculation(
        brake_system.deceleration_request, 
        brake_system.vehicle_speed
    );
    
    // Compare results (diverse implementation)
    deviation_percent = fabs(primary_result - secondary_result) / 
                       max(primary_result, secondary_result) * 100.0f;
    
    if (deviation_percent > SAFETY_MARGIN_PERCENT) {
        // Calculation mismatch detected
        report_safety_fault(FAULT_CALCULATION_MISMATCH);
        activate_emergency_brake();
        return E_NOT_OK;
    }
    
    // Monitor each brake channel
    for (int i = 0; i < BRAKE_CHANNELS; i++) {
        if (monitor_brake_channel(&brake_system.channels[i]) != E_OK) {
            handle_channel_fault(i);
        }
    }
    
    // Cross-check with independent sensors
    if (!verify_deceleration_plausibility()) {
        activate_emergency_brake();
        return E_NOT_OK;
    }
    
    return E_OK;
}

static void activate_emergency_brake(void)
{
    brake_system.emergency_brake_active = true;
    
    // Activate mechanical backup system
    engage_mechanical_brake_backup();
    
    // Notify other safety systems
    notify_safety_systems(EMERGENCY_BRAKE_ACTIVE);
    
    // Log safety event
    log_safety_event(EVENT_EMERGENCY_BRAKE, get_system_time());
}
\`\`\`

Safety features implemented:
• Dual-channel diverse calculations
• Continuous plausibility monitoring
• Mechanical backup system integration
• Comprehensive fault detection and handling
• ASIL D compliance verification
• Independent safety monitoring`
        ];
        
        this.snippets = {
            'rtos-template': `#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define TASK_PRIORITY       (2)
#define TASK_STACK_SIZE     (1024)

static void sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms
    
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        // Read sensor data
        uint16_t sensor_value = read_sensor();
        
        // Process data
        process_sensor_data(sensor_value);
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void create_sensor_task(void)
{
    xTaskCreate(sensor_task, "SensorTask", 
                TASK_STACK_SIZE, NULL, 
                TASK_PRIORITY, NULL);
}`,
            'autosar-rte': `/* AUTOSAR RTE Generated Code */
#include "Rte_SensorComponent.h"

FUNC(void, SensorComponent_CODE) SensorComponent_Init(void)
{
    /* Initialize sensor hardware */
    sensor_hardware_init();
    
    /* Set default values */
    Rte_IWrite_SensorComponent_SensorValue(0);
}

FUNC(void, SensorComponent_CODE) SensorComponent_Runnable(void)
{
    uint16 sensorData;
    Std_ReturnType status;
    
    /* Read from sensor */
    status = sensor_read(&sensorData);
    
    if (status == E_OK)
    {
        /* Write to RTE */
        Rte_IWrite_SensorComponent_SensorValue(sensorData);
        
        /* Trigger events */
        Rte_Call_SensorPort_TriggerEvent();
    }
}`,
            'safety-checklist': `Safety Requirements Checklist:

✓ Hazard Analysis and Risk Assessment (HARA) completed
✓ ASIL classification determined
✓ Safety goals defined and allocated
✓ Functional safety requirements specified
✓ Hardware-software interface defined
✓ Diagnostic coverage requirements met
✓ Fault detection and handling implemented
✓ Safety mechanisms verified
✓ Verification and validation plan executed
✓ Documentation and traceability maintained`,
            'can-driver': `#include "can_driver.h"
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef hcan1;

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
} can_message_t;

HAL_StatusTypeDef can_init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 16;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Configure filter */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    /* Start CAN */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}`
        };
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.setupThemeToggle();
        this.setupAccordion();
        this.setupChatInput();
    }
    
    setupEventListeners() {
        // Send button click
        document.getElementById('sendBtn').addEventListener('click', () => {
            this.sendMessage();
        });
        
        // Enter key in chat input
        document.getElementById('chatInput').addEventListener('keydown', (e) => {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                this.sendMessage();
            }
        });
        
        // Insert to Chat buttons
        document.querySelectorAll('.insert-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const snippetId = e.target.getAttribute('data-snippet');
                this.insertSnippet(snippetId);
            });
        });
    }
    
    setupThemeToggle() {
        const themeToggle = document.getElementById('themeToggle');
        const themeIcon = themeToggle.querySelector('.theme-icon');
        
        themeToggle.addEventListener('click', () => {
            this.currentTheme = this.currentTheme === 'light' ? 'dark' : 'light';
            document.documentElement.setAttribute('data-color-scheme', this.currentTheme);
            themeIcon.textContent = this.currentTheme === 'light' ? '🌙' : '☀️';
        });
    }
    
    setupAccordion() {
        document.querySelectorAll('.accordion-header').forEach(header => {
            header.addEventListener('click', () => {
                const target = header.getAttribute('data-target');
                const content = document.getElementById(target);
                const icon = header.querySelector('.accordion-icon');
                
                // Toggle active state
                const isActive = content.classList.contains('active');
                
                // Close all other accordions
                document.querySelectorAll('.accordion-content').forEach(item => {
                    item.classList.remove('active');
                });
                document.querySelectorAll('.accordion-header').forEach(h => {
                    h.setAttribute('aria-expanded', 'false');
                });
                
                // Toggle current accordion
                if (!isActive) {
                    content.classList.add('active');
                    header.setAttribute('aria-expanded', 'true');
                } else {
                    content.classList.remove('active');
                    header.setAttribute('aria-expanded', 'false');
                }
            });
        });
    }
    
    setupChatInput() {
        const chatInput = document.getElementById('chatInput');
        
        // Auto-resize textarea
        chatInput.addEventListener('input', () => {
            chatInput.style.height = 'auto';
            chatInput.style.height = Math.min(chatInput.scrollHeight, 120) + 'px';
        });
    }
    
    sendMessage() {
        const chatInput = document.getElementById('chatInput');
        const message = chatInput.value.trim();
        
        if (!message || this.isTyping) return;
        
        // Add user message
        this.addMessage('user', message);
        
        // Clear input
        chatInput.value = '';
        chatInput.style.height = 'auto';
        
        // Show typing indicator and simulate AI response
        this.showTypingIndicator();
        
        setTimeout(() => {
            this.hideTypingIndicator();
            this.addAIResponse();
        }, 1000 + Math.random() * 2000); // Random delay between 1-3 seconds
    }
    
    addMessage(sender, text) {
        this.messages.push({ sender, text });
        this.renderMessages();
    }
    
    addAIResponse() {
        const randomResponse = this.demoResponses[Math.floor(Math.random() * this.demoResponses.length)];
        this.addMessage('ai', randomResponse);
    }
    
    showTypingIndicator() {
        this.isTyping = true;
        const chatMessages = document.getElementById('chatMessages');
        
        const typingDiv = document.createElement('div');
        typingDiv.className = 'typing-indicator';
        typingDiv.id = 'typingIndicator';
        typingDiv.innerHTML = `
            <span>AI is thinking</span>
            <div class="typing-dots">
                <span></span>
                <span></span>
                <span></span>
            </div>
        `;
        
        chatMessages.appendChild(typingDiv);
        chatMessages.scrollTop = chatMessages.scrollHeight;
    }
    
    hideTypingIndicator() {
        this.isTyping = false;
        const typingIndicator = document.getElementById('typingIndicator');
        if (typingIndicator) {
            typingIndicator.remove();
        }
    }
    
    renderMessages() {
        const chatMessages = document.getElementById('chatMessages');
        
        // Clear existing messages except welcome
        const welcomeMessage = chatMessages.querySelector('.welcome-message');
        chatMessages.innerHTML = '';
        if (welcomeMessage) {
            chatMessages.appendChild(welcomeMessage);
        }
        
        // Render all messages
        this.messages.forEach(message => {
            const messageDiv = document.createElement('div');
            messageDiv.className = `message ${message.sender}-message`;
            
            const contentDiv = document.createElement('div');
            contentDiv.className = 'message-content';
            
            // Process markdown-like formatting
            const processedText = this.processMarkdown(message.text);
            contentDiv.innerHTML = processedText;
            
            messageDiv.appendChild(contentDiv);
            chatMessages.appendChild(messageDiv);
        });
        
        // Scroll to bottom
        chatMessages.scrollTop = chatMessages.scrollHeight;
    }
    
    processMarkdown(text) {
        // Process code blocks
        text = text.replace(/```(\w+)?\n([\s\S]*?)```/g, (match, lang, code) => {
            return `<pre><code class="language-${lang || 'text'}">${this.escapeHtml(code.trim())}</code></pre>`;
        });
        
        // Process inline code
        text = text.replace(/`([^`]+)`/g, '<code>$1</code>');
        
        // Process bullet points
        text = text.replace(/^• (.+)$/gm, '<li>$1</li>');
        text = text.replace(/(<li>.*<\/li>)/s, '<ul>$1</ul>');
        
        // Process paragraphs
        text = text.replace(/\n\n/g, '</p><p>');
        text = text.replace(/\n/g, '<br>');
        
        // Wrap in paragraph tags if not already wrapped
        if (!text.startsWith('<') && !text.includes('</p>')) {
            text = '<p>' + text + '</p>';
        }
        
        return text;
    }
    
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
    
    insertSnippet(snippetId) {
        const chatInput = document.getElementById('chatInput');
        const snippetText = this.snippets[snippetId];
        
        if (snippetText) {
            chatInput.value = snippetText;
            chatInput.focus();
            
            // Auto-resize textarea
            chatInput.style.height = 'auto';
            chatInput.style.height = Math.min(chatInput.scrollHeight, 120) + 'px';
            
            // Close accordion
            document.querySelectorAll('.accordion-content').forEach(item => {
                item.classList.remove('active');
            });
            document.querySelectorAll('.accordion-header').forEach(h => {
                h.setAttribute('aria-expanded', 'false');
            });
        }
    }
}

// Initialize the application when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    new EmbeddedAICopilot();
});