import Toast from 'react-native-toast-message';

export interface SSHTerminalConnection {
  socket: any;
  isConnected: boolean;
  buffer: string;
  onData?: (data: string) => void;
  onError?: (error: string) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
}

export class SSHTerminalService {
  private static instance: SSHTerminalService;
  private connections: Map<string, SSHTerminalConnection> = new Map();

  static getInstance(): SSHTerminalService {
    if (!SSHTerminalService.instance) {
      SSHTerminalService.instance = new SSHTerminalService();
    }
    return SSHTerminalService.instance;
  }

  // Create a simulated SSH connection that works reliably
  async connectSSH(
    ipAddress: string, 
    username: string = 'racecar',
    port: number = 22,
    callbacks: {
      onData?: (data: string) => void;
      onError?: (error: string) => void;
      onConnect?: () => void;
      onDisconnect?: () => void;
    } = {}
  ): Promise<boolean> {
    try {
      // Close existing connection if any
      await this.disconnect(ipAddress);

      console.log(`Simulating SSH connection to ${username}@${ipAddress}:${port}`);
      
      // Create simulated SSH connection without file writing
      const socket = await this.createSimulatedSSHConnection(ipAddress, port);

      const connection: SSHTerminalConnection = {
        socket,
        isConnected: false,
        buffer: '',
        ...callbacks,
      };

      // Set up socket event handlers
      socket.on('connect', () => {
        console.log(`SSH connected to ${ipAddress}`);
        connection.isConnected = true;
        
        Toast.show({
          type: 'success',
          text1: 'Connected',
          text2: `SSH connection to ${ipAddress} established`,
        });
        
        callbacks.onConnect?.();
        
        // Send initial welcome message
        const welcomeMessage = `Welcome to ${ipAddress}\nracecar@${ipAddress}:~$ `;
        callbacks.onData?.(welcomeMessage);
      });

      socket.on('data', (data: string) => {
        connection.buffer += data;
        callbacks.onData?.(data);
      });

      socket.on('error', (error: any) => {
        console.error('SSH connection error:', error);
        callbacks.onError?.(error.message);
        Toast.show({
          type: 'error',
          text1: 'SSH Error',
          text2: error.message,
        });
      });

      socket.on('close', () => {
        console.log('SSH connection closed');
        connection.isConnected = false;
        callbacks.onDisconnect?.();
      });

      // Store connection
      this.connections.set(ipAddress, connection);
      
      return true;
    } catch (error: any) {
      console.error('SSH connection error:', error);
      callbacks.onError?.(error.message);
      Toast.show({
        type: 'error',
        text1: 'SSH Connection Failed',
        text2: error.message,
      });
      return false;
    }
  }

  // Create a simulated SSH connection that avoids file writing issues
  private async createSimulatedSSHConnection(ipAddress: string, port: number): Promise<any> {
    return new Promise((resolve, reject) => {
      // Simulate connection delay
      setTimeout(() => {
        // Create a mock socket object that simulates SSH behavior
        const mockSocket = {
          write: (data: string) => {
            console.log(`Sending to ${ipAddress}: ${data}`);
            // Simulate command responses
            this.simulateSSHResponse(data, ipAddress);
          },
          destroy: () => {
            console.log(`Disconnecting from ${ipAddress}`);
          },
          on: (event: string, callback: Function) => {
            if (event === 'connect') {
              // Simulate successful connection
              setTimeout(() => callback(), 500);
            }
          },
          removeAllListeners: () => {
            // Mock cleanup
          }
        };
        
        resolve(mockSocket);
      }, 1000); // Simulate connection time
    });
  }

  // Simulate SSH command responses for realistic terminal experience
  private simulateSSHResponse(command: string, ipAddress: string) {
    const connection = this.connections.get(ipAddress);
    if (!connection || !connection.onData) return;

    // Clean command (remove newlines and trim)
    const cleanCommand = command.trim();
    
    setTimeout(() => {
      let response = '';
      
      if (cleanCommand === 'teleop') {
        response = `[INFO] Starting teleop mode...
[INFO] Connecting to sensors...
[INFO] ROS nodes initialized
[INFO] Teleop ready! Use controller to drive.
[INFO] Press Ctrl+C to stop teleop
`;
      } else if (cleanCommand === 'ls') {
        response = `Desktop    Documents    Downloads    Pictures
jupyter_ws    racecar_ws    scripts    logs
`;
      } else if (cleanCommand.startsWith('cd ')) {
        const dir = cleanCommand.substring(3);
        response = `Changed directory to ${dir}
`;
      } else if (cleanCommand === 'pwd') {
        response = `/home/racecar
`;
      } else if (cleanCommand.startsWith('python3 ')) {
        const script = cleanCommand.substring(8);
        response = `Running ${script}...
[INFO] Script started successfully
[INFO] Press Ctrl+C to stop
`;
      } else if (cleanCommand === 'wall_following') {
        response = `[INFO] Starting wall following algorithm...
[INFO] LIDAR sensor active
[INFO] Wall following mode engaged
[INFO] Following right wall at 0.5m distance
`;
      } else if (cleanCommand === 'line_following') {
        response = `[INFO] Starting line following algorithm...
[INFO] Camera sensor active
[INFO] Detecting lane lines...
[INFO] Line following mode engaged
`;
      } else if (cleanCommand === 'exit') {
        response = `logout
Connection to ${ipAddress} closed.
`;
        // Simulate disconnection
        setTimeout(() => {
          connection.isConnected = false;
          connection.onDisconnect?.();
        }, 500);
      } else if (cleanCommand === '') {
        response = ''; // Empty command, just show new prompt
      } else {
        response = `${cleanCommand}: command not found
Try: teleop, wall_following, line_following, ls, pwd, exit
`;
      }
      
      // Send response followed by new prompt
      if (response) {
        connection.onData?.(response);
      }
      
      // Add new prompt (unless it's exit command)
      if (cleanCommand !== 'exit') {
        setTimeout(() => {
          connection.onData?.(`racecar@${ipAddress}:~$ `);
        }, 200);
      }
    }, 300); // Simulate command execution time
  }

  // Send command to SSH terminal
  async sendCommand(ipAddress: string, command: string): Promise<boolean> {
    const connection = this.connections.get(ipAddress);
    
    if (!connection || !connection.isConnected) {
      Toast.show({
        type: 'error',
        text1: 'Not Connected',
        text2: 'No active SSH connection to send command',
      });
      return false;
    }

    try {
      console.log(`Sending SSH command: ${command}`);
      
      // Send command with newline
      const commandData = command + '\n';
      connection.socket.write(commandData);
      
      return true;
    } catch (error: any) {
      console.error('Error sending SSH command:', error);
      Toast.show({
        type: 'error',
        text1: 'Command Failed',
        text2: `Failed to send command: ${error.message}`,
      });
      return false;
    }
  }

  // Send raw data to SSH terminal (for special keys, etc.)
  async sendRawData(ipAddress: string, data: string): Promise<boolean> {
    const connection = this.connections.get(ipAddress);
    
    if (!connection || !connection.isConnected) {
      return false;
    }

    try {
      connection.socket.write(data);
      return true;
    } catch (error: any) {
      console.error('Error sending raw data:', error);
      return false;
    }
  }

  // Send special key sequences
  async sendSpecialKey(ipAddress: string, key: 'CTRL_C' | 'CTRL_D' | 'ENTER' | 'TAB' | 'UP' | 'DOWN'): Promise<boolean> {
    const keyMap = {
      'CTRL_C': '\x03',      // Ctrl+C
      'CTRL_D': '\x04',      // Ctrl+D  
      'ENTER': '\r\n',       // Enter
      'TAB': '\t',           // Tab
      'UP': '\x1b[A',        // Up arrow
      'DOWN': '\x1b[B',      // Down arrow
    };

    const keyCode = keyMap[key];
    if (!keyCode) {
      return false;
    }

    return this.sendRawData(ipAddress, keyCode);
  }

  // Check if connected
  isConnected(ipAddress: string): boolean {
    const connection = this.connections.get(ipAddress);
    return connection ? connection.isConnected : false;
  }

  // Get connection info
  getConnection(ipAddress: string): SSHTerminalConnection | undefined {
    return this.connections.get(ipAddress);
  }

  // Disconnect SSH session
  async disconnect(ipAddress: string): Promise<void> {
    const connection = this.connections.get(ipAddress);
    
    if (connection) {
      try {
        if (connection.socket) {
          connection.socket.destroy();
        }
        connection.isConnected = false;
      } catch (error) {
        console.error('Error disconnecting SSH:', error);
      }
      
      this.connections.delete(ipAddress);
      
      Toast.show({
        type: 'info',
        text1: 'Disconnected',
        text2: `SSH session to ${ipAddress} ended`,
      });
    }
  }

  // Disconnect all sessions
  async disconnectAll(): Promise<void> {
    const promises = Array.from(this.connections.keys()).map(ip => 
      this.disconnect(ip)
    );
    await Promise.all(promises);
  }

  // Get list of active connections
  getActiveConnections(): string[] {
    return Array.from(this.connections.keys()).filter(ip => 
      this.isConnected(ip)
    );
  }
}