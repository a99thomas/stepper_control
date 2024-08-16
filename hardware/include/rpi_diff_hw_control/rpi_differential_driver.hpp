/**
 * @author: Dharan Kumar Nallagatla 
*/

#ifndef RPI_DIFFERENTIAL_DRIVER_H
#define RPI_DIFFERENTIAL_DRIVER_H


#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <pigpiod_if2.h> 
// #include <lgpio.h>



class RpiDriveController : public rclcpp::Node
{
    private:
        
        /* Constants */ 
        // const unsigned int m_leftEn = 13;  // GPIO pin number
        // const unsigned int m_rightEn = 12; // GPIO pin number
        // const unsigned int m_leftBackward = 6; // GPIO pin number
        // const unsigned int m_leftForward = 16; // GPIO pin number
        // const unsigned int m_rightForward = 16; // GPIO pin number
        // const unsigned int m_rightBackward = 20; // GPIO pin number


        // const unsigned int m_left_dir = 19;
        // const unsigned int m_right_dir = 6;
        // const unsigned int m_left_step = 13;
        // const unsigned int m_right_step = 5;


        const unsigned int m_left_dir = 16;
        const unsigned int m_right_dir = 20;
        const unsigned int m_left_step = 19;
        const unsigned int m_right_step = 17;
        const unsigned int m_en = 25;

        const unsigned int m_motorRpm = 120;              // max rpm of motor on full voltage 
        const double m_wheelDiameter = 0.075;      // in meters
        const double m_wheelSeparation = 0.22;     // in meters
        const int m_maxPwmval = 100;           // 100 for Raspberry Pi, 255 for Arduino
        const int m_minPwmVal = 0;             // Minimum PWM value that is needed for the robot to move

        const int m_maxfreq = 6400;


        const double m_wheelRadius = m_wheelDiameter / 2;
        const double m_circumference_of_wheel = 2 * M_PI * m_wheelRadius;
        const double m_maxSpeed = (m_circumference_of_wheel * m_motorRpm) / 60;   // m/sec

        double lspeedfreq = 0;
        double rspeedfreq = 0;

        /* PWM frequency is 100 Hz */ 
        const unsigned int pwmFrequency = 100;

        int m_pwmL = 0, m_pwmR = 0;

        int m_isConnectionOk = 0; // pigpiod connection check
   
    public:
        
        /**
         * @brief: RpiDriveController Constructor
        */
        RpiDriveController(std::string node_name): Node(node_name)
        {
            RCLCPP_INFO(this->get_logger(), "RpiDriveController Constructor Is Invoked");

            initParameters();
        }

        /**
         * @brief: RpiDriveController Destructor(Returns Non-Zero Value if Connection is Success)
        */
        ~RpiDriveController()
        {
            RCLCPP_INFO(this->get_logger(), "RpiDriveController Destructor Is Invoked");
            pigpio_stop(this->m_isConnectionOk);
        }

        /**
         * @brief: Initializing The Parameters
        */
        void initParameters()
        {
            // Printing initialization parameters
            RCLCPP_INFO(get_logger(), "AMR RpiDriveController Initialized with following Params-");
            RCLCPP_INFO(get_logger(), "Motor Max RPM:\t%d RPM", this->m_motorRpm);
            RCLCPP_INFO(get_logger(), "Wheel Diameter:\t%f m", this->m_wheelDiameter);
            RCLCPP_INFO(get_logger(), "Wheel Separation:\t%f m", this->m_wheelSeparation);
            RCLCPP_INFO(get_logger(), "Robot Max Speed:\t%f m/sec", this->m_maxSpeed);
        }

        /**
         * @brief: Stop The AMR
        */
        void stop()
        {
            if (this->m_isConnectionOk >= 0) 
            {

                lspeedfreq = 0;
                rspeedfreq = 0;
                gpio_write(this->m_isConnectionOk, this->m_en, PI_HIGH);
                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_left_step, 0);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_left_dir, PI_LOW);

                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_right_step, 0);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_right_dir, PI_LOW);
            }
        }

        /**
         * @brief: Move Forward 
        */
        void forward(double left_speed, double right_speed)
        {
            if (this->m_isConnectionOk >= 0) 
            {
                RCLCPP_INFO(get_logger(),"Left speed value = %.2f", left_speed);
                RCLCPP_INFO(get_logger()," Right speed value = %.2f", right_speed);
                RCLCPP_INFO(get_logger()," Max speed value = %.2f", m_maxSpeed);
                RCLCPP_INFO(get_logger()," Max PWM speed value = %d", m_maxPwmval);
                RCLCPP_INFO(get_logger()," MIn PWM speed value = %d", m_minPwmVal);

                // double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                // double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                
                lspeedfreq = std::max(std::min(((left_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));
                rspeedfreq = std::max(std::min(((right_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));

                RCLCPP_INFO(get_logger(),"LPWM value = %.2f", lspeedfreq);
                RCLCPP_INFO(get_logger(),"RightPWM value = %.2f", rspeedfreq);
                
                // Set PWM duty cycle for the GPIO pin
                gpio_write(this->m_isConnectionOk, this->m_en, PI_LOW);
                
                set_PWM_dutycycle(m_isConnectionOk,this->m_left_step, 50);
                set_PWM_dutycycle(m_isConnectionOk,this->m_right_step, 50);


                set_PWM_frequency(m_isConnectionOk,this->m_left_step, lspeedfreq);
                set_PWM_frequency(m_isConnectionOk,this->m_right_step, rspeedfreq);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_left_dir, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_right_dir, PI_LOW);
            }   
        }

        /**
         * @brief: Move backward 
        */
        void backward(double left_speed, double right_speed)
        {
            if (m_isConnectionOk >= 0) 
            {            
                // double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                // double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
          
                lspeedfreq = std::max(std::min(((left_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));
                rspeedfreq = std::max(std::min(((right_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));

                RCLCPP_INFO(get_logger(),"LPWM value = %.2f", lspeedfreq);
                RCLCPP_INFO(get_logger(),"RightPWM value = %.2f", rspeedfreq);
                // Set PWM duty cycle for the GPIO pin
                gpio_write(this->m_isConnectionOk, this->m_en, PI_LOW);

                set_PWM_dutycycle(m_isConnectionOk,this->m_left_step, 50);
                set_PWM_dutycycle(m_isConnectionOk,this->m_right_step, 50);

                set_PWM_frequency(m_isConnectionOk,this->m_left_step, lspeedfreq);
                set_PWM_frequency(m_isConnectionOk,this->m_right_step, rspeedfreq);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_left_dir, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_right_dir, PI_HIGH);
            }
        }

        /**
         * @brief: Move Left 
        */
        void left(double left_speed, double right_speed)
        {
            if (m_isConnectionOk >= 0) 
            { 
                // double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                // double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
          
                lspeedfreq = std::max(std::min(((left_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));
                rspeedfreq = std::max(std::min(((right_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));

                RCLCPP_INFO(get_logger(),"LPWM value = %.2f", lspeedfreq);
                RCLCPP_INFO(get_logger(),"RightPWM value = %.2f", rspeedfreq);
                // Set PWM duty cycle for the GPIO pin

                gpio_write(this->m_isConnectionOk, this->m_en, PI_LOW);

                set_PWM_dutycycle(m_isConnectionOk,this->m_left_step, 50);
                set_PWM_dutycycle(m_isConnectionOk,this->m_right_step, 50);

                set_PWM_frequency(m_isConnectionOk,this->m_left_step, lspeedfreq);
                set_PWM_frequency(m_isConnectionOk,this->m_right_step, rspeedfreq);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_left_dir, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_right_dir, PI_LOW);
            }
        }

        /**
         * @brief: Move Right 
        */
        void right(double left_speed, double right_speed)
        {
            if (m_isConnectionOk >= 0) 
            {
                // double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                // double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
          
                lspeedfreq = std::max(std::min(((left_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));
                rspeedfreq = std::max(std::min(((right_speed / m_maxSpeed) * m_maxfreq), static_cast<double>(m_maxfreq)), static_cast<double>(0));


                RCLCPP_INFO(get_logger(),"LPWM value = %.2f", lspeedfreq);
                RCLCPP_INFO(get_logger(),"RightPWM value = %.2f", rspeedfreq);

                gpio_write(this->m_isConnectionOk, this->m_en, PI_LOW);

                set_PWM_dutycycle(m_isConnectionOk,this->m_left_step, 50);
                set_PWM_dutycycle(m_isConnectionOk,this->m_right_step, 50);

                set_PWM_frequency(m_isConnectionOk,this->m_left_step, lspeedfreq);
                set_PWM_frequency(m_isConnectionOk,this->m_right_step, rspeedfreq);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_left_dir, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_right_dir, PI_HIGH);
            }
        }

        bool startRpi()
        {
            RCLCPP_INFO(get_logger()," StartRpi Function Is Invoked");
            this->m_isConnectionOk = pigpio_start(NULL, NULL); // Checking The Pigpiod Connection (Returns Non-Zero Value if Connection is Success)

            if (this->m_isConnectionOk < 0)
            {
                RCLCPP_ERROR(get_logger()," Pigpio Initialization Failed");
                return false;
            }
            else
            {
                RCLCPP_WARN(get_logger(),"Pigpio Initialization Successful");

                RCLCPP_INFO(this->get_logger(), "Setting All The Pin Mode To Output");
                   
                set_mode(this->m_isConnectionOk, this->m_left_step, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_right_step, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_left_dir, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_right_dir, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_en, PI_OUTPUT);
                gpio_write(this->m_isConnectionOk, this->m_en, PI_HIGH);

                /* Set GPIO pin as PWM output and start PWM with specified frequency */
                this->m_pwmL = set_PWM_frequency(this->m_isConnectionOk, this->m_left_step, this->pwmFrequency);
                set_PWM_dutycycle(m_isConnectionOk,this->m_left_step, 0);

                /* Set GPIO pin as PWM output and start PWM with specified frequency */
                this->m_pwmR = set_PWM_frequency(this->m_isConnectionOk, this->m_right_step, this->pwmFrequency);
                set_PWM_dutycycle(m_isConnectionOk,this->m_right_step, 0);

                return true;
            }
        }

        bool stopRpi()
        {
            RCLCPP_INFO(get_logger()," StopRpi Function Is Invoked");
            pigpio_stop(this->m_isConnectionOk);
            return true;
        }

        int readFreq(std::string wheel_name)
        {
            RCLCPP_INFO(get_logger()," Read Encoder Function For %s wheel Is Invoked ", wheel_name.c_str());

            int level = 0;

            if(wheel_name == "left" || "LEFT")
            {
                /* Read the Value of the GPIO pin */
                if (gpio_read(this->m_isConnectionOk, this->m_left_dir) == 1){
                    level = -lspeedfreq;
                }
                else{ 
                    level = lspeedfreq;}
                // level = gpio_read(this->m_isConnectionOk, this->m_left_step);
                return level;
            }
            if(wheel_name == "right" || "RIGHT")
            {
                /* Read the Value of the GPIO pin */
                if (gpio_read(this->m_isConnectionOk, this->m_right_dir) == 1){
                    level = -rspeedfreq;}
                else{
                    level = rspeedfreq;
                }
                return level;
            } 
            return 0;
        }
};

#endif