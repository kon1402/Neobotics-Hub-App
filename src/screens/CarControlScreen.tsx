import React, {useState, useEffect, useRef} from 'react';
import {
  View,
  StyleSheet,
  ScrollView,
  Dimensions,
  Alert,
} from 'react-native';
import {
  Text,
  Button,
  Card,
  Surface,
  IconButton,
  Chip,
  ProgressBar,
  ActivityIndicator,
} from 'react-native-paper';
import {useRoute, useNavigation, RouteProp} from '@react-navigation/native';
import {StackNavigationProp} from '@react-navigation/stack';
import {SafeAreaView} from 'react-native-safe-area-context';
import Toast from 'react-native-toast-message';

import {theme} from '../theme/theme';
import {RootStackParamList} from '../../App';
import {SSHService} from '../services/SSHService';
import RealTerminalView from '../components/RealTerminalView';

type CarControlRouteProp = RouteProp<RootStackParamList, 'CarControl'>;
type CarControlNavigationProp = StackNavigationProp<
  RootStackParamList,
  'CarControl'
>;

const {width} = Dimensions.get('window');

const CarControlScreen: React.FC = () => {
  const route = useRoute<CarControlRouteProp>();
  const navigation = useNavigation<CarControlNavigationProp>();
  const {teamNumber, ipAddress} = route.params;

  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [teleopActive, setTeleopActive] = useState(false);
  const [carStatus, setCarStatus] = useState<any>(null);
  const [terminalVisible, setTerminalVisible] = useState(false);

  const sshService = SSHService.getInstance();
  const statusInterval = useRef<NodeJS.Timeout | null>(null);

  useEffect(() => {
    // Auto-connect when screen loads
    handleConnect();
    
    return () => {
      if (statusInterval.current) {
        clearInterval(statusInterval.current);
      }
      // Disconnect when leaving screen
      handleDisconnect();
    };
  }, []);

  const handleConnect = async () => {
    setIsConnecting(true);
    try {
      const success = await sshService.connect(ipAddress);
      setIsConnected(success);
      
      if (success) {
        startStatusMonitoring();
      }
    } catch (error) {
      console.error('Connection error:', error);
    } finally {
      setIsConnecting(false);
    }
  };

  const handleDisconnect = async () => {
    if (statusInterval.current) {
      clearInterval(statusInterval.current);
    }
    await sshService.disconnect(ipAddress);
    setIsConnected(false);
    setCarStatus(null);
    setTeleopActive(false);
  };

  const startStatusMonitoring = () => {
    if (statusInterval.current) {
      clearInterval(statusInterval.current);
    }
    
    statusInterval.current = setInterval(async () => {
      try {
        const status = await sshService.getCarStatus(ipAddress);
        setCarStatus(status);
      } catch (error) {
        console.error('Status monitoring error:', error);
      }
    }, 3000);
  };

  const handleTeleop = async () => {
    if (!isConnected) {
      Toast.show({
        type: 'error',
        text1: 'Not Connected',
        text2: 'Please connect to the car first',
      });
      return;
    }

    Alert.alert(
      'Start Teleop Mode',
      'This will activate all sensors and enable remote control. Make sure the area is clear.',
      [
        {text: 'Cancel', style: 'cancel'},
        {
          text: 'Start Teleop',
          onPress: async () => {
            setTeleopActive(true);
            try {
              const result = await sshService.executeCommand(ipAddress, 'teleop');
              if (result.success) {
                Toast.show({
                  type: 'success',
                  text1: 'Teleop Started',
                  text2: 'Car is ready for remote control',
                });
              } else {
                throw new Error(result.error || 'Failed to start teleop');
              }
            } catch (error) {
              setTeleopActive(false);
              Toast.show({
                type: 'error',
                text1: 'Teleop Failed',
                text2: 'Could not start teleop mode',
              });
            }
          },
        },
      ],
    );
  };

  const handleStopTeleop = () => {
    Alert.alert(
      'Stop Teleop Mode',
      'This will stop the teleop mode and disable remote control.',
      [
        {text: 'Cancel', style: 'cancel'},
        {
          text: 'Stop',
          style: 'destructive',
          onPress: () => {
            setTeleopActive(false);
            Toast.show({
              type: 'info',
              text1: 'Teleop Stopped',
              text2: 'Remote control disabled',
            });
          },
        },
      ],
    );
  };

  const runAutonomyScript = async (scriptName: string) => {
    if (!isConnected) {
      Toast.show({
        type: 'error',
        text1: 'Not Connected',
        text2: 'Please connect to the car first',
      });
      return;
    }

    Alert.alert(
      `Run ${scriptName}`,
      `Start the ${scriptName} autonomy script?`,
      [
        {text: 'Cancel', style: 'cancel'},
        {
          text: 'Run',
          onPress: async () => {
            try {
              const result = await sshService.executeCommand(
                ipAddress,
                `python3 ${scriptName}.py`,
              );
              if (result.success) {
                Toast.show({
                  type: 'success',
                  text1: 'Script Started',
                  text2: `${scriptName} is now running`,
                });
              }
            } catch (error) {
              Toast.show({
                type: 'error',
                text1: 'Script Failed',
                text2: `Could not start ${scriptName}`,
              });
            }
          },
        },
      ],
    );
  };

  const getConnectionStatusColor = () => {
    if (isConnecting) return theme.colors.connecting;
    if (isConnected) return theme.colors.connected;
    return theme.colors.disconnected;
  };

  const getConnectionStatusText = () => {
    if (isConnecting) return 'Connecting...';
    if (isConnected) return 'Connected';
    return 'Disconnected';
  };

  return (
    <SafeAreaView style={styles.container}>
      <ScrollView contentContainerStyle={styles.scrollContent}>
        {/* Connection Status Header */}
        <Surface style={styles.statusHeader} elevation={2}>
          <View style={styles.statusRow}>
            <View style={styles.carInfo}>
              <Text variant="titleLarge" style={styles.teamTitle}>
                Team {teamNumber}
              </Text>
              <Text variant="bodyMedium" style={styles.ipAddress}>
                {ipAddress}
              </Text>
            </View>
            <View style={styles.connectionStatus}>
              {isConnecting && (
                <ActivityIndicator
                  size="small"
                  color={theme.colors.connecting}
                />
              )}
              <Chip
                icon={isConnected ? 'wifi' : 'wifi-off'}
                style={[
                  styles.statusChip,
                  {backgroundColor: getConnectionStatusColor()},
                ]}
                textStyle={styles.statusChipText}>
                {getConnectionStatusText()}
              </Chip>
            </View>
          </View>
        </Surface>

        {/* Connection Controls */}
        <Card style={styles.connectionCard}>
          <Card.Content>
            <Text variant="titleMedium" style={styles.cardTitle}>
              Connection Control
            </Text>
            <View style={styles.connectionButtons}>
              <Button
                mode={isConnected ? 'outlined' : 'contained'}
                onPress={handleConnect}
                disabled={isConnecting}
                style={styles.connectionButton}
                icon="wifi">
                {isConnecting ? 'Connecting...' : 'Connect'}
              </Button>
              <Button
                mode="outlined"
                onPress={handleDisconnect}
                disabled={!isConnected}
                style={styles.connectionButton}
                icon="wifi-off">
                Disconnect
              </Button>
            </View>
          </Card.Content>
        </Card>

        {/* Car Status */}
        {isConnected && carStatus && (
          <Card style={styles.statusCard}>
            <Card.Content>
              <Text variant="titleMedium" style={styles.cardTitle}>
                Car Status
              </Text>
              
              {/* Battery */}
              <View style={styles.statusRow}>
                <Text variant="bodyMedium">Battery:</Text>
                <View style={styles.batteryContainer}>
                  <ProgressBar
                    progress={carStatus.battery / 100}
                    color={carStatus.battery > 20 ? theme.colors.success : theme.colors.error}
                    style={styles.batteryBar}
                  />
                  <Text variant="bodySmall">{carStatus.battery}%</Text>
                </View>
              </View>

              {/* Sensors */}
              <View style={styles.sensorsGrid}>
                <Chip
                  icon={carStatus.sensors.lidar ? 'check' : 'close'}
                  style={[
                    styles.sensorChip,
                    {backgroundColor: carStatus.sensors.lidar ? theme.colors.success : theme.colors.error},
                  ]}
                  textStyle={styles.sensorChipText}>
                  LIDAR
                </Chip>
                <Chip
                  icon={carStatus.sensors.camera ? 'check' : 'close'}
                  style={[
                    styles.sensorChip,
                    {backgroundColor: carStatus.sensors.camera ? theme.colors.success : theme.colors.error},
                  ]}
                  textStyle={styles.sensorChipText}>
                  Camera
                </Chip>
                <Chip
                  icon={carStatus.sensors.imu ? 'check' : 'close'}
                  style={[
                    styles.sensorChip,
                    {backgroundColor: carStatus.sensors.imu ? theme.colors.success : theme.colors.error},
                  ]}
                  textStyle={styles.sensorChipText}>
                  IMU
                </Chip>
                <Chip
                  icon={carStatus.ros ? 'check' : 'close'}
                  style={[
                    styles.sensorChip,
                    {backgroundColor: carStatus.ros ? theme.colors.success : theme.colors.error},
                  ]}
                  textStyle={styles.sensorChipText}>
                  ROS
                </Chip>
              </View>
            </Card.Content>
          </Card>
        )}

        {/* Teleop Control */}
        <Card style={styles.teleopCard}>
          <Card.Content>
            <Text variant="titleMedium" style={styles.cardTitle}>
              Remote Control
            </Text>
            <Text variant="bodyMedium" style={styles.cardDescription}>
              Start teleop mode to enable remote control of the car
            </Text>
            
            {teleopActive && (
              <Surface style={styles.teleopActiveIndicator} elevation={1}>
                <IconButton
                  icon="radio"
                  size={24}
                  iconColor={theme.colors.success}
                />
                <Text variant="bodyMedium" style={styles.teleopActiveText}>
                  Teleop mode active - Car ready for control
                </Text>
              </Surface>
            )}

            <Button
              mode={teleopActive ? 'outlined' : 'contained'}
              onPress={teleopActive ? handleStopTeleop : handleTeleop}
              disabled={!isConnected}
              style={styles.teleopButton}
              buttonColor={teleopActive ? theme.colors.error : theme.colors.primary}
              icon={teleopActive ? 'stop' : 'play'}>
              {teleopActive ? 'Stop Teleop' : 'Start Teleop'}
            </Button>
          </Card.Content>
        </Card>

        {/* Autonomy Scripts */}
        <Card style={styles.autonomyCard}>
          <Card.Content>
            <Text variant="titleMedium" style={styles.cardTitle}>
              Autonomy Scripts
            </Text>
            <Text variant="bodyMedium" style={styles.cardDescription}>
              Run autonomous navigation algorithms
            </Text>
            <View style={styles.autonomyButtons}>
              <Button
                mode="contained-tonal"
                onPress={() => runAutonomyScript('wall_following')}
                disabled={!isConnected}
                style={styles.autonomyButton}
                icon="wall">
                Wall Following
              </Button>
              <Button
                mode="contained-tonal"
                onPress={() => runAutonomyScript('line_following')}
                disabled={!isConnected}
                style={styles.autonomyButton}
                icon="road">
                Line Following
              </Button>
            </View>
          </Card.Content>
        </Card>

        {/* Terminal Access */}
        <Card style={styles.terminalCard}>
          <Card.Content>
            <Text variant="titleMedium" style={styles.cardTitle}>
              Terminal Access
            </Text>
            <Text variant="bodyMedium" style={styles.cardDescription}>
              Direct command line access to the car
            </Text>
            <Button
              mode="outlined"
              onPress={() => setTerminalVisible(true)}
              disabled={!isConnected}
              style={styles.terminalButton}
              icon="console">
              Open Terminal
            </Button>
          </Card.Content>
        </Card>
      </ScrollView>

      {/* Real SSH Terminal Modal */}
      {terminalVisible && (
        <RealTerminalView
          visible={terminalVisible}
          onClose={() => setTerminalVisible(false)}
          ipAddress={ipAddress}
          username="racecar"
        />
      )}
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: theme.colors.background,
  },
  scrollContent: {
    padding: theme.spacing.md,
  },
  statusHeader: {
    marginBottom: theme.spacing.md,
    padding: theme.spacing.lg,
    borderRadius: theme.borderRadius.md,
  },
  statusRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: theme.spacing.sm,
  },
  carInfo: {
    flex: 1,
  },
  teamTitle: {
    color: theme.colors.primary,
    fontWeight: 'bold',
  },
  ipAddress: {
    color: theme.colors.onSurfaceVariant,
    fontFamily: 'monospace',
  },
  connectionStatus: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  statusChip: {
    marginLeft: theme.spacing.sm,
  },
  statusChipText: {
    color: theme.colors.onPrimary,
    fontWeight: 'bold',
  },
  connectionCard: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  statusCard: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  teleopCard: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  autonomyCard: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  terminalCard: {
    marginBottom: theme.spacing.md,
    elevation: 2,
  },
  cardTitle: {
    marginBottom: theme.spacing.xs,
    color: theme.colors.onSurface,
  },
  cardDescription: {
    marginBottom: theme.spacing.lg,
    color: theme.colors.onSurfaceVariant,
  },
  connectionButtons: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  connectionButton: {
    flex: 1,
    marginHorizontal: theme.spacing.xs,
  },
  batteryContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
    marginLeft: theme.spacing.md,
  },
  batteryBar: {
    flex: 1,
    marginRight: theme.spacing.sm,
  },
  sensorsGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    marginTop: theme.spacing.md,
  },
  sensorChip: {
    margin: theme.spacing.xs,
  },
  sensorChipText: {
    color: theme.colors.onPrimary,
    fontWeight: 'bold',
  },
  teleopActiveIndicator: {
    flexDirection: 'row',
    alignItems: 'center',
    padding: theme.spacing.md,
    marginBottom: theme.spacing.md,
    borderRadius: theme.borderRadius.sm,
    backgroundColor: theme.colors.primaryContainer,
  },
  teleopActiveText: {
    flex: 1,
    color: theme.colors.onPrimaryContainer,
    fontWeight: '500',
  },
  teleopButton: {
    borderRadius: theme.borderRadius.md,
  },
  autonomyButtons: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  autonomyButton: {
    flex: 1,
    marginHorizontal: theme.spacing.xs,
  },
  terminalButton: {
    borderRadius: theme.borderRadius.md,
  },
});

export default CarControlScreen;
