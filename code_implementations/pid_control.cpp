#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>

class PIDController {
private:
    double kp, ki, kd;
    double integral;
    double previous_error;
    bool first_run;
    
public:
    PIDController(double p, double i, double d) 
        : kp(p), ki(i), kd(d), integral(0), previous_error(0), first_run(true) {}
    
    double calculate(double setpoint, double process_variable, double dt = 1.0) {
        double error = setpoint - process_variable;
        
        // Proportional term
        double proportional = kp * error;
        
        // Integral term (accumulate error over time)
        integral += error * dt;
        double integral_term = ki * integral;
        
        // Derivative term (rate of change of error)
        double derivative = 0;
        if (!first_run) {
            derivative = kd * (error - previous_error) / dt;
        }
        first_run = false;
        
        // Store error for next iteration
        previous_error = error;
        
        // Calculate total output
        double output = proportional + integral_term + derivative;
        
        // Optional: clamp output to reasonable range (0-100%)
        if (output < 0) output = 0;
        if (output > 100) output = 100;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        previous_error = 0;
        first_run = true;
    }
};

class TemperatureSystem {
private:
    double current_temp;
    double heat_efficiency; // degrees per percent output
    
public:
    TemperatureSystem(double initial_temp, double efficiency) 
        : current_temp(initial_temp), heat_efficiency(efficiency) {}
    
    void update(double heater_output, double dt = 1.0) {
        // Simple model: temperature change = heater_output * efficiency
        double temp_change = heater_output * heat_efficiency * dt;
        current_temp += temp_change;
    }
    
    double getTemperature() const {
        return current_temp;
    }
};

void printTableHeader() {
    std::cout << std::setw(6) << "Time" 
              << std::setw(8) << "Temp" 
              << std::setw(8) << "Error" 
              << std::setw(10) << "Output%" 
              << std::setw(12) << "Graph" << std::endl;
    std::cout << std::string(44, '-') << std::endl;
}

void printTableRow(int time, double temp, double error, double output) {
    std::cout << std::setw(6) << time 
              << std::setw(8) << std::fixed << std::setprecision(1) << temp
              << std::setw(8) << std::fixed << std::setprecision(1) << error
              << std::setw(10) << std::fixed << std::setprecision(1) << output << "%"
              << std::setw(4) << "";
    
    // Simple ASCII graph (each * represents ~2 degrees)
    int graph_pos = static_cast<int>((temp - 50) / 2); // Scale for 50-80°F range
    if (graph_pos < 0) graph_pos = 0;
    if (graph_pos > 15) graph_pos = 15;
    
    for (int i = 0; i < graph_pos; ++i) {
        std::cout << "*";
    }
    std::cout << std::endl;
}

void printGraph(const std::vector<double>& temperatures, double setpoint) {
    std::cout << "\n\nTemperature Over Time Graph:\n";
    std::cout << "50°F    55°F    60°F    65°F    70°F    75°F    80°F\n";
    std::cout << "|       |       |       |       |       |       |\n";
    
    for (size_t i = 0; i < temperatures.size(); ++i) {
        std::cout << std::setw(2) << i << ": ";
        
        // Scale temperature to graph position (50-80°F range, 30 char width)
        int pos = static_cast<int>((temperatures[i] - 50) * 30 / 30);
        if (pos < 0) pos = 0;
        if (pos > 30) pos = 30;
        
        // Show setpoint marker
        int setpoint_pos = static_cast<int>((setpoint - 50) * 30 / 30);
        
        for (int j = 0; j <= 30; ++j) {
            if (j == pos) {
                std::cout << "*";  // Current temperature
            } else if (j == setpoint_pos) {
                std::cout << "|";  // Setpoint line
            } else {
                std::cout << " ";
            }
        }
        std::cout << " (" << std::fixed << std::setprecision(1) << temperatures[i] << "°F)\n";
    }
}

int main() {
    // PID parameters (same as our paper example)
    double kp = 2.0;
    double ki = 0.1;
    double kd = 0.5;
    
    // System parameters
    double setpoint = 70.0;  // Target temperature
    double initial_temp = 60.0;
    double heat_efficiency = 0.1;  // 0.1°F per 1% heater output
    
    // Create controller and system
    PIDController pid(kp, ki, kd);
    TemperatureSystem system(initial_temp, heat_efficiency);
    
    // Simulation parameters
    int simulation_time = 20;  // 20 time steps
    std::vector<double> temperature_history;
    
    std::cout << "PID Temperature Control Simulation\n";
    std::cout << "Setpoint: " << setpoint << "°F\n";
    std::cout << "PID Parameters: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << "\n\n";
    
    printTableHeader();
    
    // Run simulation
    for (int t = 0; t < simulation_time; ++t) {
        double current_temp = round(system.getTemperature()); // Round like our paper example
        double error = setpoint - current_temp;
        double output = pid.calculate(setpoint, current_temp);
        
        // Store temperature for graphing
        temperature_history.push_back(current_temp);
        
        // Print current state
        printTableRow(t, current_temp, error, output);
        
        // Update system for next iteration
        system.update(output);
        
        // Stop if we're very close to setpoint for several iterations
        if (std::abs(error) < 0.5) {
            static int close_count = 0;
            close_count++;
            if (close_count > 3) {
                std::cout << "\nSetpoint reached! Stopping simulation.\n";
                break;
            }
        }
    }
    
    // Print final graph
    printGraph(temperature_history, setpoint);
    
    std::cout << "\nSimulation complete!\n";
    std::cout << "Final temperature: " << std::fixed << std::setprecision(1) 
              << system.getTemperature() << "°F\n";
    
    return 0;
}