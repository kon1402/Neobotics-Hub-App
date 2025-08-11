import {Alert} from 'react-native';
import Toast from 'react-native-toast-message';
import {SSHTerminalService} from './SSHTerminalService';

export interface ConnectionStatus {
  isConnected: boolean;
  isConnecting: boolean;
  error?: string;
}

export interface CarInfo {
  teamNumber: number;
  ipAddress: string;
  connectionStatus: ConnectionStatus;
}

export class SSHService {
  private static instance: SSHService;
  private connections: Map<string, any> = new Map();
  private terminalService: SSHTerminalService;

  static getInstance(): SSHService {
    if (!SSHService.instance) {
      SSHService.instance = new SSHService();
    }
    return SSHService.instance;
  }

  constructor() {
    this.terminalService = SSHTerminalService.getInstance();
  }

  // Real SSH connection implementation using terminal service
  async connect(ipAddress: string, username: string = 'racecar', password: string = ''): Promise<boolean> {
    try {
      // Show connecting status
      Toast.show({
        type: 'info',
        text1: 'Connecting...',
        text2: `Establishing SSH connection to ${username}@${ipAddress}`,
      });

      // Use the real terminal service for SSH connection
      const success = await this.terminalService.connectSSH(ipAddress, username, 22, {
        onConnect: () => {
          console.log(`SSH connection established to ${ipAddress}`);
        },
        onError: (error: string) => {
          console.error(`SSH connection error: ${error}`);
        },
        onDisconnect: () => {
          console.log(`SSH connection to ${ipAddress} disconnected`);
          this.connections.delete(ipAddress);
        },
      });

      if (success) {
        // Store connection info
        this.connections.set(ipAddress, {
          host: ipAddress,
          username,
          connected: true,
          connectedAt: new Date(),
          lastActivity: new Date(),
          isRealSSH: true, // Flag to indicate this is a real SSH connection
        });

        return true;
      } else {
        return false;
      }
    } catch (error: any) {
      console.error('SSH Connection Error:', error);
      
      Toast.show({
        type: 'error',
        text1: 'Connection Failed',
        text2: error.message || 'Failed to establish SSH connection',
      });
      
      return false;
    }
  }

  // Test network reachability to the target IP
  private async testNetworkReachability(ipAddress: string): Promise<boolean> {
    try {
      // Use fetch with a timeout to test reachability
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), 5000);
      
      // Try to connect to port 22 (SSH) or just ping the host
      // Note: This is a simplified test - in production you'd use a proper network library
      const response = await fetch(`http://${ipAddress}:22`, {
        method: 'HEAD',
        signal: controller.signal,
        mode: 'no-cors', // Important for cross-origin requests
      }).catch(() => null);
      
      clearTimeout(timeoutId);
      
      // Even if fetch fails, the host might be reachable for SSH
      // So we'll return true and let the SSH attempt handle the real connection
      return true;
    } catch (error) {
      return false;
    }
  }

  // Attempt actual SSH connection
  private async attemptSSHConnection(ipAddress: string, username: string): Promise<{success: boolean; error?: string}> {
    try {
      // In a real React Native app, you would use:
      // 1. react-native-ssh2 
      // 2. A custom native module
      // 3. A WebSocket bridge to a Node.js SSH service
      
      // For now, we'll simulate a real SSH attempt with proper error scenarios
      await new Promise(resolve => setTimeout(resolve, 3000)); // Connection attempt delay
      
      // Simulate real SSH connection scenarios based on IP pattern
      const teamNumber = this.extractTeamNumber(ipAddress);
      
      // Simulate different failure scenarios for demo purposes
      // In real implementation, these would be actual SSH connection results
      if (teamNumber > 99) {
        return { success: false, error: 'Invalid team number. Must be 1-99.' };
      }
      
      // Simulate network issues for certain IPs (for demo)
      if (teamNumber % 20 === 0) {
        return { success: false, error: 'Connection timeout. Car may be offline.' };
      }
      
      if (teamNumber % 15 === 0) {
        return { success: false, error: 'Host unreachable. Check network connection.' };
      }
      
      if (teamNumber % 10 === 0) {
        return { success: false, error: 'Connection refused. SSH service may not be running.' };
      }
      
      // Most connections should succeed for demo
      return { success: true };
      
    } catch (error: any) {
      return { success: false, error: error.message || 'SSH connection failed' };
    }
  }

  // Extract team number from IP address
  private extractTeamNumber(ipAddress: string): number {
    const parts = ipAddress.split('.');
    if (parts.length === 4 && parts[0] === '192' && parts[1] === '168' && parts[2] === '1') {
      const lastOctet = parseInt(parts[3], 10);
      if (lastOctet >= 101 && lastOctet <= 199) {
        return lastOctet - 100;
      }
    }
    return -1;
  }

  async disconnect(ipAddress: string): Promise<void> {
    try {
      // Disconnect from real SSH terminal if connected
      await this.terminalService.disconnect(ipAddress);
      
      // Remove from our connection tracking
      this.connections.delete(ipAddress);
      
      Toast.show({
        type: 'info',
        text1: 'Disconnected',
        text2: `SSH session to ${ipAddress} ended`,
      });
    } catch (error) {
      console.error('Disconnect error:', error);
    }
  }

  isConnected(ipAddress: string): boolean {
    return this.connections.has(ipAddress);
  }

  async executeCommand(ipAddress: string, command: string): Promise<{success: boolean; output: string; error?: string}> {
    if (!this.isConnected(ipAddress)) {
      return {
        success: false,
        output: '',
        error: 'Not connected to car. Please establish SSH connection first.',
      };
    }

    try {
      console.log(`Executing SSH command on ${ipAddress}: ${command}`);
      
      // Show command execution status
      Toast.show({
        type: 'info',
        text1: 'Executing Command',
        text2: `Running: ${command}`,
      });

      // Simulate real SSH command execution with proper delay
      await new Promise(resolve => setTimeout(resolve, 1500));

      // In a real implementation, you would execute:
      // ssh racecar@${ipAddress} "${command}"
      
      // For now, simulate realistic command responses
      const result = await this.simulateSSHCommand(ipAddress, command);
      
      if (result.success) {
        Toast.show({
          type: 'success',
          text1: 'Command Executed',
          text2: 'Command completed successfully',
        });
      } else {
        Toast.show({
          type: 'error',
          text1: 'Command Failed',
          text2: result.error || 'Command execution failed',
        });
      }

      return result;
    } catch (error: any) {
      console.error('SSH Command Error:', error);
      return {
        success: false,
        output: '',
        error: `SSH command failed: ${error.message}`,
      };
    }
  }

  // Simulate SSH command execution with realistic responses
  private async simulateSSHCommand(ipAddress: string, command: string): Promise<{success: boolean; output: string; error?: string}> {
    const cmd = command.trim();
    
    // Add realistic timing based on command type
    if (cmd === 'teleop') {
      await new Promise(resolve => setTimeout(resolve, 2000)); // Teleop takes longer to start
    }
    
    switch (cmd) {
      case 'teleop':
        return {
          success: true,
          output: `racecar@${ipAddress}:~$ teleop
[INFO] Starting teleop mode...
[INFO] Initializing ROS nodes...
[INFO] Camera node: OK
[INFO] LIDAR node: OK  
[INFO] IMU node: OK
[INFO] Motor controller: OK
[INFO] Joy node: OK
[INFO] Teleop ready! Use controller to drive.
[INFO] Press Ctrl+C to stop teleop mode.
Press 'q' to quit`,
        };
        
      case 'ls':
        return {
          success: true,
          output: `jupyter_ws  racecar_ws  logs  scripts  Desktop  Documents`,
        };
        
      case 'cd racecar_ws && ls':
        return {
          success: true,
          output: `build  install  log  src  devel`,
        };
        
      case 'python3 wall_following.py':
        return {
          success: true,
          output: `racecar@${ipAddress}:~$ python3 wall_following.py
[INFO] Initializing wall following algorithm...
[INFO] Subscribing to /scan topic...
[INFO] LIDAR data received
[INFO] Wall detected at distance: 1.2m
[INFO] Calculating steering angle...
[INFO] Publishing cmd_vel: linear=0.5, angular=-0.3
[INFO] Wall following active. Press Ctrl+C to stop.`,
        };
        
      case 'python3 line_following.py':
        return {
          success: true,
          output: `racecar@${ipAddress}:~$ python3 line_following.py
[INFO] Initializing line following algorithm...
[INFO] Camera node ready
[INFO] Image processing started
[INFO] Line detected with confidence: 89%
[INFO] Line center offset: -12 pixels
[INFO] Steering adjustment: 15 degrees left
[INFO] Line following active. Press Ctrl+C to stop.`,
        };
        
      case 'rostopic list':
        return {
          success: true,
          output: `/camera/image_raw
/camera/camera_info
/cmd_vel
/drive
/imu/data
/joy
/scan
/tf
/tf_static
/vesc/sensors/core
/vesc/sensors/servo_position_command`,
        };
        
      case 'rosnode list':
        return {
          success: true,
          output: `/camera_node
/imu_node
/joy_node
/lidar_node  
/motor_controller
/teleop_node
/vesc_driver`,
        };
        
      case 'pwd':
        return {
          success: true,
          output: `/home/racecar`,
        };
        
      case 'whoami':
        return {
          success: true,
          output: `racecar`,
        };
        
      case 'date':
        return {
          success: true,
          output: new Date().toString(),
        };
        
      default:
        // Handle unknown commands
        if (cmd.startsWith('cd ')) {
          return {
            success: true,
            output: ``, // cd typically has no output
          };
        } else if (cmd.includes('&&')) {
          return {
            success: false,
            error: `Complex commands not supported in this demo. Try individual commands.`,
            output: '',
          };
        } else {
          return {
            success: false,
            error: `bash: ${cmd}: command not found`,
            output: '',
          };
        }
    }
  }

  // Simulate checking car status
  async getCarStatus(ipAddress: string): Promise<{
    sensors: {lidar: boolean; camera: boolean; imu: boolean};
    ros: boolean;
    teleop: boolean;
    battery: number;
  }> {
    if (!this.isConnected(ipAddress)) {
      throw new Error('Not connected');
    }

    // Mock status data
    return {
      sensors: {
        lidar: Math.random() > 0.1,
        camera: Math.random() > 0.05,
        imu: Math.random() > 0.02,
      },
      ros: Math.random() > 0.1,
      teleop: Math.random() > 0.7,
      battery: Math.floor(Math.random() * 30) + 70, // 70-100%
    };
  }

  getConnectionInfo(ipAddress: string): any {
    return this.connections.get(ipAddress);
  }
}
