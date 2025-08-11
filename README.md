# Neobotics Hub App

A revolutionary mobile application that enables seamless control of autonomous racing cars from the convenience of your phone or tablet. Built for educational institutions and racing enthusiasts who want to experience cutting-edge robotics technology.

## Overview

The Neobotics Hub App provides a comprehensive interface for managing and controlling 1/14th scale autonomous racing cars. Whether you're a student learning robotics, an educator teaching autonomous systems, or an enthusiast exploring the future of racing, this app bridges the gap between complex robotics and intuitive mobile control.

## Key Features

### Vehicle Management
- **Multi-Vehicle Support**: Connect to up to 99 racing cars simultaneously
- **Team-Based Organization**: Assign cars to teams (1-99) with automatic IP mapping
- **Real-Time Status Monitoring**: Monitor connection health and vehicle states
- **Quick Vehicle Selection**: Streamlined interface for selecting and connecting to vehicles

### Remote Control & Automation
- **Teleop Mode**: Direct manual control of vehicles using intuitive touch controls
- **Autonomous Modes**: 
  - Wall Following: Navigate using LiDAR sensors for obstacle avoidance
  - Line Following: Camera-based navigation following track markers
- **Command Terminal**: Full SSH terminal access for advanced users
- **Script Execution**: Run custom Python scripts and ROS commands

### Professional Interface
- **Neobotics Branding**: Professional design using official color schemes
  - Winning Red (#EB341C)
  - Tire Marks (#1D1D27) 
  - Circuit Steel (#BFBFBF)
- **Responsive Design**: Optimized for both phones and tablets
- **Real-Time Feedback**: Live command output and system responses
- **Intuitive Navigation**: Hub-based interface for quick access to all features

## Technical Architecture

### Mobile App (React Native)
- **Cross-Platform**: Runs on iOS and Android devices
- **Real-Time Communication**: SSH-based connection to racing cars
- **Responsive UI**: Material Design components with custom theming
- **State Management**: Efficient handling of multiple vehicle connections

### Vehicle Integration
- **Network Protocol**: Static IP addressing (192.168.1.101-199)
- **SSH Communication**: Secure shell access for command execution
- **ROS Integration**: Compatible with Robot Operating System
- **Sensor Support**: LiDAR, cameras, and other autonomous navigation sensors

### Development Features
- **Hot Reloading**: Fast development cycle with Metro bundler
- **Modular Architecture**: Clean separation of concerns
- **TypeScript Support**: Type-safe development environment
- **Professional Debugging**: Comprehensive logging and error handling

## Getting Started

### Prerequisites
- Node.js 18 or higher
- React Native development environment
- iOS development tools (Xcode) or Android Studio
- Access to Neobotics racing car network

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/your-username/neobotics-hub-app.git
   cd neobotics-hub-app/NeoboticsCarApp
   ```

2. **Install dependencies**
   ```bash
   npm install
   ```

3. **iOS Setup**
   ```bash
   cd ios
   pod install
   cd ..
   ```

4. **Start Metro bundler**
   ```bash
   npm start
   ```

5. **Run the app**
   ```bash
   # For iOS
   npx react-native run-ios
   
   # For Android
   npx react-native run-android
   ```

### Configuration

The app automatically detects and connects to Neobotics racing cars on the network. Ensure your device is connected to the same network as the racing cars (IP range: 192.168.1.101-199).

## Usage Guide

### Basic Operation

1. **Launch the App**: Open Neobotics Hub on your device
2. **Select Vehicle**: Choose "NeoRacer" from the vehicle selection screen
3. **Choose Team**: Enter your team number (1-99) to connect to the assigned car
4. **Connect**: Tap "Connect" to establish SSH connection to your racing car
5. **Control**: Use the interface to run teleop mode or autonomous scripts

### Advanced Features

#### Terminal Access
- Access full SSH terminal for advanced command execution
- Run custom Python scripts and ROS commands
- Monitor real-time system output and logs

#### Autonomous Modes
- **Wall Following**: `python3 wall_following.py`
- **Line Following**: `python3 line_following.py`
- **Custom Scripts**: Execute any Python script in the racing car environment

#### Multi-Team Management
- Connect to multiple cars simultaneously
- Switch between active connections
- Monitor status of all connected vehicles

## Development

### Project Structure
```
NeoboticsCarApp/
├── src/
│   ├── components/          # Reusable UI components
│   ├── screens/            # Main app screens
│   ├── services/           # Business logic and APIs
│   ├── theme/              # Design system and styling
│   └── navigation/         # App navigation setup
├── ios/                    # iOS-specific files
├── android/               # Android-specific files
└── assets/                # Images and static resources
```

### Key Components
- **HomeScreen**: Main hub interface
- **TeamSelectionScreen**: Vehicle connection setup
- **CarControlScreen**: Remote control interface
- **TerminalView**: SSH terminal component
- **SSHTerminalService**: Communication layer

### Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## Technology Stack

- **Frontend**: React Native, TypeScript
- **UI Framework**: React Native Paper (Material Design)
- **Navigation**: React Navigation
- **State Management**: React Context + Hooks
- **Communication**: SSH protocol
- **Build System**: Metro bundler, Xcode, Gradle

## Educational Applications

### For Students
- Learn autonomous vehicle programming
- Understand sensor integration and data processing
- Experience real-world robotics applications
- Develop mobile app interfaces for hardware control

### For Educators
- Demonstrate autonomous systems concepts
- Teach network communication protocols
- Show practical applications of computer vision and LiDAR
- Facilitate hands-on robotics competitions

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For technical support, feature requests, or educational partnerships:
- Create an issue on GitHub
- Contact Neobotics support team
- Visit our documentation wiki

## Roadmap

### Upcoming Features
- Real-time telemetry visualization
- Race performance analytics
- Multi-car coordination modes
- Enhanced debugging tools
- Cloud-based car management

### Long-term Vision
- Support for full-scale autonomous vehicles
- Integration with professional racing telemetry
- AI-powered driving assistance
- Global competition platform

---

**Neobotics Hub App** - Bringing the future of autonomous racing to your fingertips.