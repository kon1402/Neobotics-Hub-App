import React, {useState, useEffect, useRef} from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  TextInput,
  Dimensions,
  Modal,
  Alert,
} from 'react-native';
import {
  Text,
  Surface,
  IconButton,
  Button,
  Chip,
} from 'react-native-paper';
import {SafeAreaView} from 'react-native-safe-area-context';

import {theme} from '../theme/theme';
import {SSHTerminalService} from '../services/SSHTerminalService';

interface RealTerminalViewProps {
  visible: boolean;
  onClose: () => void;
  ipAddress: string;
  username?: string;
}

interface TerminalLine {
  id: string;
  content: string;
  timestamp: Date;
  isCommand?: boolean;
}

const RealTerminalView: React.FC<RealTerminalViewProps> = ({
  visible,
  onClose,
  ipAddress,
  username = 'racecar',
}) => {
  const [lines, setLines] = useState<TerminalLine[]>([]);
  const [currentCommand, setCurrentCommand] = useState('');
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [connectionAttempted, setConnectionAttempted] = useState(false);

  const scrollViewRef = useRef<ScrollView>(null);
  const inputRef = useRef<TextInput>(null);
  const terminalService = SSHTerminalService.getInstance();

  const commonCommands = [
    'teleop',
    'ls',
    'pwd',
    'cd racecar_ws',
    'python3 wall_following.py',
    'python3 line_following.py',
    'rostopic list',
    'rosnode list',
    'htop',
    'exit',
  ];

  useEffect(() => {
    if (visible && !connectionAttempted) {
      connectToSSH();
      setConnectionAttempted(true);
    } else if (!visible) {
      // Reset state when modal closes
      setConnectionAttempted(false);
      setIsConnected(false);
      setLines([]);
      setCurrentCommand('');
    }

    return () => {
      if (!visible) {
        disconnectSSH();
      }
    };
  }, [visible]);

  const addLine = (content: string, isCommand: boolean = false) => {
    const newLine: TerminalLine = {
      id: Date.now().toString() + Math.random().toString(),
      content,
      timestamp: new Date(),
      isCommand,
    };
    
    setLines(prev => [...prev, newLine]);
    
    // Auto-scroll to bottom
    setTimeout(() => {
      scrollViewRef.current?.scrollToEnd({animated: true});
    }, 100);
  };

  const connectToSSH = async () => {
    setIsConnecting(true);
    addLine(`Connecting to ${username}@${ipAddress}...`, false);

    try {
      const success = await terminalService.connectSSH(ipAddress, username, 22, {
        onConnect: () => {
          setIsConnected(true);
          setIsConnecting(false);
          addLine(`Connected to ${username}@${ipAddress}`, false);
          addLine(`${username}@${ipAddress}:~$ `, false);
          
          // Focus input after connection
          setTimeout(() => {
            inputRef.current?.focus();
          }, 500);
        },
        onError: (error: string) => {
          setIsConnected(false);
          setIsConnecting(false);
          addLine(`Connection failed: ${error}`, false);
        },
        onDisconnect: () => {
          setIsConnected(false);
          addLine('Connection closed', false);
        },
        onData: (data: string) => {
          // Add real-time data from SSH session
          if (data.trim()) {
            addLine(data, false);
          }
        },
      });

      if (!success) {
        setIsConnecting(false);
        addLine('Failed to establish SSH connection', false);
      }
    } catch (error: any) {
      setIsConnecting(false);
      addLine(`Connection error: ${error.message}`, false);
    }
  };

  const disconnectSSH = async () => {
    if (isConnected) {
      await terminalService.disconnect(ipAddress);
      setIsConnected(false);
      addLine('Disconnected from SSH session', false);
    }
  };

  const executeCommand = async () => {
    if (!currentCommand.trim() || !isConnected) return;

    const command = currentCommand.trim();
    
    // Add command to terminal display
    addLine(`${username}@${ipAddress}:~$ ${command}`, true);
    
    // Send command to real SSH session
    const success = await terminalService.sendCommand(ipAddress, command);
    
    if (!success) {
      addLine('Error: Failed to send command to SSH session', false);
    }

    // Clear input
    setCurrentCommand('');
  };

  const sendSpecialKey = async (key: 'CTRL_C' | 'CTRL_D' | 'TAB' | 'UP' | 'DOWN') => {
    if (!isConnected) return;
    
    const keyNames = {
      'CTRL_C': 'Ctrl+C',
      'CTRL_D': 'Ctrl+D',
      'TAB': 'Tab',
      'UP': 'Up',
      'DOWN': 'Down',
    };
    
    addLine(`[${keyNames[key]}]`, false);
    await terminalService.sendSpecialKey(ipAddress, key);
  };

  const insertCommand = (command: string) => {
    setCurrentCommand(command);
    inputRef.current?.focus();
  };

  const clearTerminal = () => {
    setLines([]);
    if (isConnected) {
      addLine(`${username}@${ipAddress}:~$ `, false);
    }
  };

  const handleReconnect = () => {
    Alert.alert(
      'Reconnect',
      'Disconnect and reconnect to the SSH session?',
      [
        {text: 'Cancel', style: 'cancel'},
        {
          text: 'Reconnect',
          onPress: async () => {
            await disconnectSSH();
            setTimeout(() => {
              setConnectionAttempted(false);
              connectToSSH();
            }, 1000);
          },
        },
      ],
    );
  };

  return (
    <Modal
      visible={visible}
      animationType="slide"
      presentationStyle="fullScreen">
      <SafeAreaView style={styles.container}>
        {/* Header */}
        <Surface style={styles.header} elevation={2}>
          <View style={styles.headerContent}>
            <View style={styles.headerInfo}>
              <Text variant="titleMedium" style={styles.headerTitle}>
                SSH Terminal - {ipAddress}
              </Text>
              <Text variant="bodySmall" style={styles.headerSubtitle}>
                Real SSH session to {username}@{ipAddress}
              </Text>
            </View>
            <View style={styles.headerActions}>
              <Chip
                icon={isConnected ? 'wifi' : isConnecting ? 'wifi-arrow-up-down' : 'wifi-off'}
                style={[
                  styles.statusChip,
                  {backgroundColor: isConnected ? theme.colors.success : isConnecting ? theme.colors.warning : theme.colors.error},
                ]}
                textStyle={styles.statusChipText}>
                {isConnected ? 'Connected' : isConnecting ? 'Connecting...' : 'Disconnected'}
              </Chip>
              <IconButton
                icon="refresh"
                size={24}
                onPress={handleReconnect}
                iconColor={theme.colors.onSurface}
                disabled={isConnecting}
              />
              <IconButton
                icon="broom"
                size={24}
                onPress={clearTerminal}
                iconColor={theme.colors.onSurface}
              />
              <IconButton
                icon="close"
                size={24}
                onPress={onClose}
                iconColor={theme.colors.onSurface}
              />
            </View>
          </View>
        </Surface>

        {/* Common Commands */}
        {isConnected && (
          <ScrollView
            horizontal
            style={styles.commandsBar}
            showsHorizontalScrollIndicator={false}>
            {commonCommands.map((cmd, index) => (
              <Chip
                key={index}
                onPress={() => insertCommand(cmd)}
                style={styles.commandChip}
                textStyle={styles.commandChipText}>
                {cmd}
              </Chip>
            ))}
          </ScrollView>
        )}

        {/* Terminal Output */}
        <ScrollView
          ref={scrollViewRef}
          style={styles.terminalContainer}
          contentContainerStyle={styles.terminalContent}>
          {lines.map(line => (
            <Text
              key={line.id}
              style={[
                styles.terminalLine,
                line.isCommand ? styles.commandLine : styles.outputLine,
              ]}
              selectable>
              {line.content}
            </Text>
          ))}
        </ScrollView>

        {/* Control Panel */}
        {isConnected && (
          <Surface style={styles.controlPanel} elevation={2}>
            <ScrollView
              horizontal
              showsHorizontalScrollIndicator={false}
              style={styles.controlsScroll}>
              <Button
                mode="outlined"
                onPress={() => sendSpecialKey('CTRL_C')}
                style={styles.controlButton}
                compact>
                Ctrl+C
              </Button>
              <Button
                mode="outlined"
                onPress={() => sendSpecialKey('CTRL_D')}
                style={styles.controlButton}
                compact>
                Ctrl+D
              </Button>
              <Button
                mode="outlined"
                onPress={() => sendSpecialKey('TAB')}
                style={styles.controlButton}
                compact>
                Tab
              </Button>
              <Button
                mode="outlined"
                onPress={() => sendSpecialKey('UP')}
                style={styles.controlButton}
                compact>
                ↑
              </Button>
              <Button
                mode="outlined"
                onPress={() => sendSpecialKey('DOWN')}
                style={styles.controlButton}
                compact>
                ↓
              </Button>
            </ScrollView>
          </Surface>
        )}

        {/* Input Area */}
        {isConnected && (
          <Surface style={styles.inputContainer} elevation={4}>
            <View style={styles.inputRow}>
              <Text style={styles.prompt}>$</Text>
              <TextInput
                ref={inputRef}
                style={styles.input}
                value={currentCommand}
                onChangeText={setCurrentCommand}
                onSubmitEditing={executeCommand}
                placeholder="Type a command..."
                placeholderTextColor={theme.colors.onSurfaceVariant}
                autoCorrect={false}
                autoCapitalize="none"
                multiline={false}
                returnKeyType="send"
              />
              <IconButton
                icon="send"
                size={20}
                onPress={executeCommand}
                disabled={!currentCommand.trim()}
                iconColor={theme.colors.winningRed}
              />
            </View>
          </Surface>
        )}

        {/* Connection Status */}
        {!isConnected && !isConnecting && (
          <Surface style={styles.disconnectedPanel} elevation={2}>
            <Text variant="bodyMedium" style={styles.disconnectedText}>
              SSH connection is not active
            </Text>
            <Button
              mode="contained"
              onPress={handleReconnect}
              style={styles.reconnectButton}
              buttonColor={theme.colors.winningRed}>
              Reconnect
            </Button>
          </Surface>
        )}
      </SafeAreaView>
    </Modal>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1a1a1a', // Dark terminal background
  },
  header: {
    backgroundColor: theme.colors.tireMarks,
    paddingVertical: theme.spacing.sm,
    paddingHorizontal: theme.spacing.md,
  },
  headerContent: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  headerInfo: {
    flex: 1,
  },
  headerTitle: {
    color: theme.colors.winningRed,
    fontWeight: 'bold',
  },
  headerSubtitle: {
    color: theme.colors.circuitSteel,
  },
  headerActions: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  statusChip: {
    marginRight: theme.spacing.sm,
  },
  statusChipText: {
    color: theme.colors.onPrimary,
    fontSize: 12,
    fontWeight: 'bold',
  },
  commandsBar: {
    backgroundColor: theme.colors.tireMarks,
    paddingVertical: theme.spacing.sm,
    paddingHorizontal: theme.spacing.md,
  },
  commandChip: {
    marginRight: theme.spacing.sm,
    backgroundColor: theme.colors.circuitSteel,
  },
  commandChipText: {
    fontSize: 12,
    fontFamily: 'monospace',
    color: theme.colors.tireMarks,
  },
  terminalContainer: {
    flex: 1,
    backgroundColor: '#1a1a1a',
  },
  terminalContent: {
    padding: theme.spacing.md,
  },
  terminalLine: {
    fontFamily: 'monospace',
    fontSize: 14,
    lineHeight: 20,
    marginBottom: 2,
  },
  commandLine: {
    color: theme.colors.winningRed, // Red for commands
  },
  outputLine: {
    color: '#FFFFFF', // White for output
  },
  controlPanel: {
    backgroundColor: theme.colors.surfaceVariant,
    paddingVertical: theme.spacing.sm,
  },
  controlsScroll: {
    paddingHorizontal: theme.spacing.md,
  },
  controlButton: {
    marginRight: theme.spacing.sm,
    borderColor: theme.colors.winningRed,
  },
  inputContainer: {
    backgroundColor: theme.colors.tireMarks,
    paddingHorizontal: theme.spacing.md,
    paddingVertical: theme.spacing.sm,
  },
  inputRow: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  prompt: {
    fontFamily: 'monospace',
    fontSize: 16,
    color: theme.colors.winningRed,
    marginRight: theme.spacing.sm,
    fontWeight: 'bold',
  },
  input: {
    flex: 1,
    fontFamily: 'monospace',
    fontSize: 16,
    color: theme.colors.circuitSteel,
    paddingVertical: theme.spacing.sm,
    paddingHorizontal: theme.spacing.sm,
    backgroundColor: 'transparent',
    borderWidth: 0,
  },
  disconnectedPanel: {
    backgroundColor: theme.colors.surfaceVariant,
    padding: theme.spacing.lg,
    alignItems: 'center',
  },
  disconnectedText: {
    color: theme.colors.onSurfaceVariant,
    marginBottom: theme.spacing.md,
    textAlign: 'center',
  },
  reconnectButton: {
    borderRadius: theme.borderRadius.md,
  },
});

export default RealTerminalView;
