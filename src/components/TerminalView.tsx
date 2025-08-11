import React, {useState, useEffect, useRef} from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  TextInput,
  Dimensions,
  Keyboard,
  Modal,
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
import {SSHService} from '../services/SSHService';

interface TerminalViewProps {
  visible: boolean;
  onClose: () => void;
  ipAddress: string;
  sshService: SSHService;
}

interface TerminalLine {
  id: string;
  type: 'command' | 'output' | 'error';
  content: string;
  timestamp: Date;
}

const TerminalView: React.FC<TerminalViewProps> = ({
  visible,
  onClose,
  ipAddress,
  sshService,
}) => {
  const [lines, setLines] = useState<TerminalLine[]>([]);
  const [currentCommand, setCurrentCommand] = useState('');
  const [commandHistory, setCommandHistory] = useState<string[]>([]);
  const [historyIndex, setHistoryIndex] = useState(-1);
  const [isExecuting, setIsExecuting] = useState(false);

  const scrollViewRef = useRef<ScrollView>(null);
  const inputRef = useRef<TextInput>(null);

  const commonCommands = [
    'teleop',
    'ls',
    'cd racecar_ws',
    'python3 wall_following.py',
    'python3 line_following.py',
    'rostopic list',
    'rosnode list',
  ];

  useEffect(() => {
    if (visible) {
      // Add welcome message
      addLine({
        type: 'output',
        content: `Connected to Team Car at ${ipAddress}
racecar@${ipAddress}:~$ `,
      });
      
      // Focus input
      setTimeout(() => {
        inputRef.current?.focus();
      }, 500);
    } else {
      // Clear terminal when closed
      setLines([]);
      setCurrentCommand('');
      setHistoryIndex(-1);
    }
  }, [visible, ipAddress]);

  const addLine = (params: Omit<TerminalLine, 'id' | 'timestamp'>) => {
    const newLine: TerminalLine = {
      ...params,
      id: Date.now().toString() + Math.random().toString(),
      timestamp: new Date(),
    };
    
    setLines(prev => [...prev, newLine]);
    
    // Auto-scroll to bottom
    setTimeout(() => {
      scrollViewRef.current?.scrollToEnd({animated: true});
    }, 100);
  };

  const executeCommand = async () => {
    if (!currentCommand.trim() || isExecuting) return;

    const command = currentCommand.trim();
    
    // Add command to history
    setCommandHistory(prev => [...prev, command]);
    setHistoryIndex(-1);
    
    // Add command line
    addLine({
      type: 'command',
      content: `racecar@${ipAddress}:~$ ${command}`,
    });

    // Clear input
    setCurrentCommand('');
    setIsExecuting(true);

    try {
      // Execute command via SSH service
      const result = await sshService.executeCommand(ipAddress, command);
      
      if (result.success) {
        addLine({
          type: 'output',
          content: result.output,
        });
      } else {
        addLine({
          type: 'error',
          content: result.error || 'Command failed',
        });
      }
    } catch (error) {
      addLine({
        type: 'error',
        content: `Error: ${error}`,
      });
    } finally {
      setIsExecuting(false);
      
      // Add new prompt
      addLine({
        type: 'output',
        content: `racecar@${ipAddress}:~$ `,
      });
    }
  };

  const handleKeyPress = (key: string) => {
    if (key === 'Enter') {
      executeCommand();
    }
  };

  const navigateHistory = (direction: 'up' | 'down') => {
    if (commandHistory.length === 0) return;

    let newIndex = historyIndex;
    
    if (direction === 'up') {
      newIndex = historyIndex < commandHistory.length - 1 ? historyIndex + 1 : historyIndex;
    } else {
      newIndex = historyIndex > -1 ? historyIndex - 1 : -1;
    }
    
    setHistoryIndex(newIndex);
    
    if (newIndex === -1) {
      setCurrentCommand('');
    } else {
      setCurrentCommand(commandHistory[commandHistory.length - 1 - newIndex]);
    }
  };

  const insertCommand = (command: string) => {
    setCurrentCommand(command);
    inputRef.current?.focus();
  };

  const clearTerminal = () => {
    setLines([]);
    addLine({
      type: 'output',
      content: `racecar@${ipAddress}:~$ `,
    });
  };

  const getLineStyle = (type: TerminalLine['type']) => {
    switch (type) {
      case 'command':
        return [styles.terminalLine, styles.commandLine];
      case 'error':
        return [styles.terminalLine, styles.errorLine];
      default:
        return [styles.terminalLine, styles.outputLine];
    }
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
                Terminal - {ipAddress}
              </Text>
              <Text variant="bodySmall" style={styles.headerSubtitle}>
                Direct command line access
              </Text>
            </View>
            <View style={styles.headerActions}>
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

        {/* Terminal Output */}
        <ScrollView
          ref={scrollViewRef}
          style={styles.terminalContainer}
          contentContainerStyle={styles.terminalContent}>
          {lines.map(line => (
            <Text
              key={line.id}
              style={getLineStyle(line.type)}
              selectable>
              {line.content}
            </Text>
          ))}
        </ScrollView>

        {/* Input Area */}
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
              editable={!isExecuting}
              multiline={false}
              returnKeyType="send"
            />
            <View style={styles.inputActions}>
              <IconButton
                icon="arrow-up"
                size={20}
                onPress={() => navigateHistory('up')}
                disabled={commandHistory.length === 0}
              />
              <IconButton
                icon="arrow-down"
                size={20}
                onPress={() => navigateHistory('down')}
                disabled={commandHistory.length === 0}
              />
              <IconButton
                icon="send"
                size={20}
                onPress={executeCommand}
                disabled={!currentCommand.trim() || isExecuting}
                iconColor={theme.colors.primary}
              />
            </View>
          </View>
        </Surface>
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
    backgroundColor: theme.colors.surface,
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
    color: theme.colors.onSurface,
    fontWeight: 'bold',
  },
  headerSubtitle: {
    color: theme.colors.onSurfaceVariant,
  },
  headerActions: {
    flexDirection: 'row',
  },
  commandsBar: {
    backgroundColor: theme.colors.surfaceVariant,
    paddingVertical: theme.spacing.sm,
    paddingHorizontal: theme.spacing.md,
  },
  commandChip: {
    marginRight: theme.spacing.sm,
    backgroundColor: theme.colors.surface,
  },
  commandChipText: {
    fontSize: 12,
    fontFamily: 'monospace',
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
    color: '#4CAF50', // Green for commands
  },
  outputLine: {
    color: '#FFFFFF', // White for normal output
  },
  errorLine: {
    color: '#F44336', // Red for errors
  },
  inputContainer: {
    backgroundColor: theme.colors.surface,
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
    color: theme.colors.primary,
    marginRight: theme.spacing.sm,
    fontWeight: 'bold',
  },
  input: {
    flex: 1,
    fontFamily: 'monospace',
    fontSize: 16,
    color: theme.colors.onSurface,
    paddingVertical: theme.spacing.sm,
    paddingHorizontal: theme.spacing.sm,
    backgroundColor: 'transparent',
    borderWidth: 0,
  },
  inputActions: {
    flexDirection: 'row',
    alignItems: 'center',
  },
});

export default TerminalView;
