import React, {useState} from 'react';
import {
  View,
  StyleSheet,
  FlatList,
  Dimensions,
  Alert,
  ScrollView,
} from 'react-native';
import {
  Text,
  Button,
  Card,
  TextInput,
  Chip,
  Surface,
  IconButton,
} from 'react-native-paper';
import {useNavigation} from '@react-navigation/native';
import {StackNavigationProp} from '@react-navigation/stack';
import {SafeAreaView} from 'react-native-safe-area-context';
import Toast from 'react-native-toast-message';

import {theme} from '../theme/theme';
import {RootStackParamList} from '../../App';

type TeamSelectionNavigationProp = StackNavigationProp<
  RootStackParamList,
  'TeamSelection'
>;

const {width} = Dimensions.get('window');

const TeamSelectionScreen: React.FC = () => {
  const navigation = useNavigation<TeamSelectionNavigationProp>();
  const [selectedTeam, setSelectedTeam] = useState<number | null>(null);
  const [customTeam, setCustomTeam] = useState('');

  // Generate quick select options (popular team numbers)
  const quickSelectTeams = [1, 2, 3, 4, 5, 10, 15, 20, 25, 30];

  const generateIPAddress = (teamNumber: number): string => {
    return `192.168.1.${100 + teamNumber}`;
  };

  const validateTeamNumber = (team: number): boolean => {
    return team >= 1 && team <= 99;
  };

  const handleQuickSelect = (teamNumber: number) => {
    setSelectedTeam(teamNumber);
    setCustomTeam('');
  };

  const handleCustomTeamChange = (text: string) => {
    setCustomTeam(text);
    const teamNumber = parseInt(text, 10);
    if (!isNaN(teamNumber) && validateTeamNumber(teamNumber)) {
      setSelectedTeam(teamNumber);
    } else {
      setSelectedTeam(null);
    }
  };

  const handleConnect = () => {
    if (!selectedTeam || !validateTeamNumber(selectedTeam)) {
      Toast.show({
        type: 'error',
        text1: 'Invalid Team Number',
        text2: 'Please select a team number between 1 and 99',
      });
      return;
    }

    const ipAddress = generateIPAddress(selectedTeam);
    
    Alert.alert(
      'Connect to Car',
      `Team: ${selectedTeam}\\nIP: ${ipAddress}\\n\\nProceed with connection?`,
      [
        {text: 'Cancel', style: 'cancel'},
        {
          text: 'Connect',
          onPress: () => {
            navigation.navigate('CarControl', {
              teamNumber: selectedTeam,
              ipAddress,
            });
          },
        },
      ],
    );
  };

  const renderQuickSelectItem = ({item}: {item: number}) => (
    <Chip
      selected={selectedTeam === item}
      onPress={() => handleQuickSelect(item)}
      style={[
        styles.teamChip,
        selectedTeam === item && styles.selectedChip,
      ]}
      textStyle={selectedTeam === item && styles.selectedChipText}>
      Team {item}
    </Chip>
  );

  return (
    <SafeAreaView style={styles.container}>
      <ScrollView 
        style={styles.scrollView}
        contentContainerStyle={styles.scrollContent}
        showsVerticalScrollIndicator={true}
      >
        <View style={styles.content}>
        {/* Header */}
        <Surface style={styles.header} elevation={1}>
          <IconButton
            icon="car-sports"
            size={40}
            iconColor={theme.colors.winningRed}
          />
          <Text variant="headlineSmall" style={styles.title}>
            Select NeoRacer Team
          </Text>
          <Text variant="bodyMedium" style={styles.subtitle}>
            Choose your team number to connect to the corresponding NeoRacer
          </Text>
        </Surface>

        {/* Quick Select */}
        <Card style={styles.quickSelectCard}>
          <Card.Content>
            <Text variant="titleMedium" style={styles.sectionTitle}>
              Quick Select
            </Text>
            <Text variant="bodySmall" style={styles.sectionDescription}>
              Tap a team number for quick selection
            </Text>
            <FlatList
              data={quickSelectTeams}
              renderItem={renderQuickSelectItem}
              keyExtractor={item => item.toString()}
              numColumns={5}
              contentContainerStyle={styles.quickSelectGrid}
              scrollEnabled={false}
            />
          </Card.Content>
        </Card>

        {/* Custom Team Input */}
        <Card style={styles.customInputCard}>
          <Card.Content>
            <Text variant="titleMedium" style={styles.sectionTitle}>
              Custom Team Number
            </Text>
            <Text variant="bodySmall" style={styles.sectionDescription}>
              Enter any team number (1-99)
            </Text>
            <TextInput
              label="Team Number"
              value={customTeam}
              onChangeText={handleCustomTeamChange}
              keyboardType="numeric"
              maxLength={2}
              style={styles.teamInput}
              left={<TextInput.Icon icon="account-group" />}
              error={customTeam !== '' && selectedTeam === null}
            />
            {customTeam !== '' && selectedTeam === null && (
              <Text variant="bodySmall" style={styles.errorText}>
                Please enter a valid team number (1-99)
              </Text>
            )}
          </Card.Content>
        </Card>

        {/* Connection Info */}
        {selectedTeam && (
          <Card style={styles.connectionInfoCard}>
            <Card.Content>
              <Text variant="titleMedium" style={styles.sectionTitle}>
                Connection Details
              </Text>
              <View style={styles.connectionDetails}>
                <View style={styles.detailRow}>
                  <Text variant="bodyMedium" style={styles.detailLabel}>
                    Team:
                  </Text>
                  <Text variant="bodyMedium" style={styles.detailValue}>
                    {selectedTeam}
                  </Text>
                </View>
                <View style={styles.detailRow}>
                  <Text variant="bodyMedium" style={styles.detailLabel}>
                    IP Address:
                  </Text>
                  <Text variant="bodyMedium" style={styles.detailValue}>
                    {generateIPAddress(selectedTeam)}
                  </Text>
                </View>
              </View>
            </Card.Content>
          </Card>
        )}

        {/* Connect Button */}
        <Button
          mode="contained"
          onPress={handleConnect}
          disabled={!selectedTeam}
          style={[
            styles.connectButton,
            !selectedTeam && styles.disabledButton,
          ]}
          contentStyle={styles.buttonContent}
          icon="car-connected">
          Connect to Team {selectedTeam || '?'} Car
        </Button>

        {/* Info */}
        <Surface style={styles.infoCard} elevation={1}>
          <IconButton
            icon="information"
            size={24}
            iconColor={theme.colors.info}
          />
          <View style={styles.infoContent}>
            <Text variant="bodySmall" style={styles.infoText}>
              Each team car has a static IP address in the format 192.168.1.1XX
              where XX is the team number + 100.
            </Text>
          </View>
        </Surface>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: theme.colors.background,
  },
  scrollView: {
    flex: 1,
  },
  scrollContent: {
    flexGrow: 1,
    paddingBottom: theme.spacing.xl,
  },
  content: {
    paddingHorizontal: theme.spacing.lg,
    paddingVertical: theme.spacing.md,
  },
  header: {
    alignItems: 'center',
    paddingVertical: theme.spacing.xl,
    paddingHorizontal: theme.spacing.lg,
    marginBottom: theme.spacing.lg,
    borderRadius: theme.borderRadius.md,
  },
  title: {
    marginTop: theme.spacing.sm,
    marginBottom: theme.spacing.xs,
    color: theme.colors.tireMarks,
    textAlign: 'center',
    fontWeight: 'bold',
  },
  subtitle: {
    color: theme.colors.onSurfaceVariant,
    textAlign: 'center',
  },
  quickSelectCard: {
    marginBottom: theme.spacing.lg,
    elevation: 2,
  },
  customInputCard: {
    marginBottom: theme.spacing.lg,
    elevation: 2,
  },
  connectionInfoCard: {
    marginBottom: theme.spacing.lg,
    elevation: 2,
    backgroundColor: theme.colors.primaryContainer,
  },
  sectionTitle: {
    marginBottom: theme.spacing.xs,
    color: theme.colors.onSurface,
  },
  sectionDescription: {
    marginBottom: theme.spacing.lg,
    color: theme.colors.onSurfaceVariant,
  },
  quickSelectGrid: {
    justifyContent: 'space-between',
  },
  teamChip: {
    margin: theme.spacing.xs,
    flex: 1,
    marginHorizontal: 2,
  },
  selectedChip: {
    backgroundColor: theme.colors.winningRed,
  },
  selectedChipText: {
    color: theme.colors.onPrimary,
  },
  teamInput: {
    marginBottom: theme.spacing.sm,
  },
  errorText: {
    color: theme.colors.error,
    marginTop: theme.spacing.xs,
  },
  connectionDetails: {
    backgroundColor: theme.colors.surface,
    padding: theme.spacing.md,
    borderRadius: theme.borderRadius.sm,
  },
  detailRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: theme.spacing.xs,
  },
  detailLabel: {
    fontWeight: '500',
    color: theme.colors.onSurface,
  },
  detailValue: {
    color: theme.colors.winningRed,
    fontWeight: 'bold',
  },
  connectButton: {
    marginBottom: theme.spacing.lg,
    borderRadius: theme.borderRadius.md,
  },
  disabledButton: {
    opacity: 0.6,
  },
  buttonContent: {
    paddingVertical: theme.spacing.sm,
  },
  infoCard: {
    flexDirection: 'row',
    alignItems: 'center',
    padding: theme.spacing.md,
    borderRadius: theme.borderRadius.sm,
    backgroundColor: theme.colors.surfaceVariant,
  },
  infoContent: {
    flex: 1,
    marginLeft: theme.spacing.sm,
  },
  infoText: {
    color: theme.colors.onSurfaceVariant,
    lineHeight: 18,
  },
});

export default TeamSelectionScreen;
