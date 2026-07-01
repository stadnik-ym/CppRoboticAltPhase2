#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdint>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"
#include <lgpio.h>

using namespace std::chrono_literals;

class INA226 {
private:
    int i2c_file = -1;
    uint8_t addr;
    const double SHUNT_RES = 0.1; // Опір шунта в Омах

    int16_t readReg(uint8_t reg) {
        if (i2c_file < 0) return 0;
        uint8_t buf[2] = {reg, 0};
        if (write(i2c_file, &reg, 1) != 1) return 0;
        if (read(i2c_file, buf, 2) != 2) return 0;
        return (int16_t)((buf[0] << 8) | buf[1]);
    }

public:
    INA226(uint8_t address) : addr(address) {}
    
    bool openBus(const char* bus = "/dev/i2c-1") {
        i2c_file = open(bus, O_RDWR);
        if (i2c_file < 0) return false;
        if (ioctl(i2c_file, I2C_SLAVE, addr) < 0) {
            close(i2c_file);
            return false;
        }
        return true;
    }

    double getVoltage() {
        return readReg(0x02) * 0.00125; // U = Raw * 1.25 mV
    }

    double getCurrent() {
        double shunt_v = readReg(0x01) * 0.0000025; // LSB = 2.5 uV
        return shunt_v / SHUNT_RES;
    }

    void closeBus() {
        if (i2c_file >= 0) { close(i2c_file); i2c_file = -1; }
    }
    ~INA226() { closeBus(); }
};

class HardwareProtectionNode : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr telemetry_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr buzzer_pub_;

    std::unique_ptr<INA226> left_sensor_;
    std::unique_ptr<INA226> right_sensor_;

    int gpio_chip_handle = -1;
    const int PIN_ALERT = 24;  // Аварійний вхід GPIO24
    
    const double CRITICAL_CURRENT = 2.5; // Межа струму в Амперах
    bool emergency_triggered = false;

    void triggerAlarm(const std::string& reason) {
        if (emergency_triggered) return;
        emergency_triggered = true;
        
        RCLCPP_ERROR(this->get_logger(), "!!! АВАРІЯ ЗАХИСТУ: %s !!!", reason.c_str());
        
        // Відправляємо сигнал активації на баззер ноду
        auto buzzer_msg = std_msgs::msg::Bool();
        buzzer_msg.data = true;
        buzzer_pub_->publish(buzzer_msg);
    }

    void checkTelemetry() {
        if (emergency_triggered) return;

        double i_left = left_sensor_->getCurrent();
        double i_right = right_sensor_->getCurrent();
        double u_sys = left_sensor_->getVoltage();

        // Публікація сирої телеметрії для моніторингу
        auto msg = geometry_msgs::msg::Point();
        msg.x = i_left;
        msg.y = i_right;
        msg.z = u_sys;
        telemetry_pub_->publish(msg);

        // Програмний дубляж захисту
        if (i_left > CRITICAL_CURRENT) triggerAlarm("Перевантаження лівого борту (" + std::to_string(i_left) + " A)");
        if (i_right > CRITICAL_CURRENT) triggerAlarm("Перевантаження правого борту (" + std::to_string(i_right) + " A)");
    }

    static void alertCallback(int chip, int gpio, int level, uint32_t tick, void *userdata) {
        auto* node = static_cast<HardwareProtectionNode*>(userdata);
        if (level == 0) { // Переривання Active Low від ALERT
            node->triggerAlarm("Апаратне переривання на GPIO24! Клінінг або заклинювання двигунів.");
        }
    }

public:
    HardwareProtectionNode() : Node("hardware_protection_node") {
        // Зафіксовані адреси після фізичного сканування шини
        left_sensor_ = std::make_unique<INA226>(0x41);  
        right_sensor_ = std::make_unique<INA226>(0x44); 

        if (!left_sensor_->openBus() || !right_sensor_->openBus()) {
            RCLCPP_FATAL(this->get_logger(), "Помилка відкриття INA226 на I2C шині!");
            throw std::runtime_error("I2C failure");
        }

        // Налаштування переривання через lgpio для лінії ALERT
        gpio_chip_handle = lgGpiochipOpen(4); // Чип 4 для RPi 5
        if (gpio_chip_handle >= 0) {
            lgGpioClaimInput(gpio_chip_handle, 0, PIN_ALERT);
            lgGpioSetAlertIn(gpio_chip_handle, PIN_ALERT, alertCallback, LG_FALLING_EDGE, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Не вдалося ініціалізувати апаратне переривання lgpio.");
        }

        // Ініціалізація топіків (зв'язок з зовнішньою нодою баззера)
        telemetry_pub_ = this->create_publisher<geometry_msgs::msg::Point>("robot/telemetry", 10);
        buzzer_pub_    = this->create_publisher<std_msgs::msg::Bool>("buzzer", 10);
        
        timer_ = this->create_wall_timer(100ms, std::bind(&HardwareProtectionNode::checkTelemetry, this));

        RCLCPP_INFO(this->get_logger(), "Вузол захисту та телеметрії успішно інтегровано в diff_drive.");
    }

    ~HardwareProtectionNode() {
        if (gpio_chip_handle >= 0) lgGpiochipClose(gpio_chip_handle);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareProtectionNode>());
    rclcpp::shutdown();
    return 0;
}