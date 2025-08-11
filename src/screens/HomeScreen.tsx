import React from 'react';
import {
  View,
  StyleSheet,
  Dimensions,
  ScrollView,
  Image,
} from 'react-native';
import {
  Text,
  Button,
  Card,
  IconButton,
  Surface,
  Divider,
} from 'react-native-paper';
import {useNavigation} from '@react-navigation/native';
import {StackNavigationProp} from '@react-navigation/stack';
import {SafeAreaView} from 'react-native-safe-area-context';

import {theme} from '../theme/theme';
import {RootStackParamList} from '../../App';

type HomeScreenNavigationProp = StackNavigationProp<RootStackParamList, 'Home'>;

const {width} = Dimensions.get('window');

const HomeScreen: React.FC = () => {
  const navigation = useNavigation<HomeScreenNavigationProp>();

  const handleSelectVehicle = () => {
    navigation.navigate('TeamSelection');
  };

  return (
    <SafeAreaView style={styles.container}>
      <ScrollView contentContainerStyle={styles.scrollContent}>
        {/* Neobotics Header */}
        <Surface style={styles.header} elevation={3}>
          <View style={styles.headerContent}>
            <Image
              source={require('../assets/images/Neobotics_Icon_CoW.png')}
              style={styles.logoImage}
              resizeMode="contain"
            />
            <Text variant="headlineLarge" style={styles.title}>
              NEOBOTICS
            </Text>
            <Text variant="titleMedium" style={styles.subtitle}>
              Control Hub
            </Text>
          </View>
        </Surface>

        {/* Main Hub Content */}
        <View style={styles.content}>
          <Text variant="headlineSmall" style={styles.welcomeText}>
            Welcome to Neobotics Hub
          </Text>
          <Text variant="bodyLarge" style={styles.description}>
            Your central command center for autonomous vehicle control and monitoring.
          </Text>

          <Divider style={styles.divider} />

          {/* Vehicle Selection Section */}
          <View style={styles.sectionContainer}>
            <Text variant="titleLarge" style={styles.sectionTitle}>
              Select Vehicle
            </Text>
            <Text variant="bodyMedium" style={styles.sectionDescription}>
              Choose your vehicle to begin control operations
            </Text>

            {/* NeoRacer Card */}
            <Card style={styles.vehicleCard} onPress={handleSelectVehicle}>
              <Card.Content style={styles.vehicleContent}>
                <View style={styles.vehicleHeader}>
                  <View style={styles.vehicleIcon}>
                    <IconButton
                      icon="car-sports"
                      size={40}
                      iconColor={theme.colors.winningRed}
                    />
                  </View>
                  <View style={styles.vehicleInfo}>
                    <Text variant="titleLarge" style={styles.vehicleTitle}>
                      NeoRacer
                    </Text>
                    <Text variant="bodyMedium" style={styles.vehicleSubtitle}>
                      1/14th Scale Autonomous Vehicle
                    </Text>
                  </View>
                  <IconButton
                    icon="chevron-right"
                    size={24}
                    iconColor={theme.colors.tireMarks}
                  />
                </View>
                
                <View style={styles.vehicleFeatures}>
                  <View style={styles.featureRow}>
                    <Text style={styles.featureBullet}>•</Text>
                    <Text variant="bodySmall" style={styles.featureText}>
                      Remote teleop control
                    </Text>
                  </View>
                  <View style={styles.featureRow}>
                    <Text style={styles.featureBullet}>•</Text>
                    <Text variant="bodySmall" style={styles.featureText}>
                      Autonomous navigation
                    </Text>
                  </View>
                  <View style={styles.featureRow}>
                    <Text style={styles.featureBullet}>•</Text>
                    <Text variant="bodySmall" style={styles.featureText}>
                      Real-time sensor monitoring
                    </Text>
                  </View>
                </View>

                <Button
                  mode="contained"
                  onPress={handleSelectVehicle}
                  style={styles.selectButton}
                  contentStyle={styles.buttonContent}
                  buttonColor={theme.colors.winningRed}
                  icon="arrow-right">
                  Select NeoRacer
                </Button>
              </Card.Content>
            </Card>
          </View>

          {/* Quick Access Section */}
          <View style={styles.sectionContainer}>
            <Text variant="titleLarge" style={styles.sectionTitle}>
              Quick Access
            </Text>
            
            <View style={styles.quickAccessGrid}>
              <Card style={styles.quickAccessCard}>
                <Card.Content style={styles.quickAccessContent}>
                  <IconButton
                    icon="lightning-bolt"
                    size={32}
                    iconColor={theme.colors.winningRed}
                  />
                  <Text variant="titleSmall" style={styles.quickAccessTitle}>
                    Teleop
                  </Text>
                  <Text variant="bodySmall" style={styles.quickAccessDesc}>
                    Direct control
                  </Text>
                </Card.Content>
              </Card>

              <Card style={styles.quickAccessCard}>
                <Card.Content style={styles.quickAccessContent}>
                  <IconButton
                    icon="brain"
                    size={32}
                    iconColor={theme.colors.winningRed}
                  />
                  <Text variant="titleSmall" style={styles.quickAccessTitle}>
                    Autonomy
                  </Text>
                  <Text variant="bodySmall" style={styles.quickAccessDesc}>
                    AI scripts
                  </Text>
                </Card.Content>
              </Card>

              <Card style={styles.quickAccessCard}>
                <Card.Content style={styles.quickAccessContent}>
                  <IconButton
                    icon="monitor-dashboard"
                    size={32}
                    iconColor={theme.colors.winningRed}
                  />
                  <Text variant="titleSmall" style={styles.quickAccessTitle}>
                    Monitor
                  </Text>
                  <Text variant="bodySmall" style={styles.quickAccessDesc}>
                    Status & logs
                  </Text>
                </Card.Content>
              </Card>

              <Card style={styles.quickAccessCard}>
                <Card.Content style={styles.quickAccessContent}>
                  <IconButton
                    icon="console"
                    size={32}
                    iconColor={theme.colors.winningRed}
                  />
                  <Text variant="titleSmall" style={styles.quickAccessTitle}>
                    Terminal
                  </Text>
                  <Text variant="bodySmall" style={styles.quickAccessDesc}>
                    Command line
                  </Text>
                </Card.Content>
              </Card>
            </View>
          </View>

          {/* Status Info */}
          <Surface style={styles.statusInfo} elevation={1}>
            <View style={styles.statusRow}>
              <IconButton
                icon="information"
                size={20}
                iconColor={theme.colors.tireMarks}
              />
              <Text variant="bodySmall" style={styles.statusText}>
                Ready to connect to NeoRacer vehicles. Select a vehicle above to begin.
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
  scrollContent: {
    flexGrow: 1,
  },
  header: {
    backgroundColor: theme.colors.tireMarks,
    paddingVertical: theme.spacing.xl,
    marginBottom: theme.spacing.lg,
  },
  headerContent: {
    alignItems: 'center',
  },
  logoImage: {
    width: 80,
    height: 80,
    marginBottom: theme.spacing.md,
  },
  title: {
    color: theme.colors.winningRed,
    fontWeight: 'bold',
    marginBottom: theme.spacing.xs,
    letterSpacing: 2,
  },
  subtitle: {
    color: theme.colors.circuitSteel,
    fontWeight: '500',
  },
  content: {
    paddingHorizontal: theme.spacing.lg,
    paddingBottom: theme.spacing.xl,
  },
  welcomeText: {
    textAlign: 'center',
    marginBottom: theme.spacing.md,
    color: theme.colors.tireMarks,
    fontWeight: '600',
  },
  description: {
    textAlign: 'center',
    marginBottom: theme.spacing.lg,
    color: theme.colors.onSurfaceVariant,
    lineHeight: 24,
  },
  divider: {
    marginVertical: theme.spacing.lg,
    backgroundColor: theme.colors.circuitSteel,
  },
  sectionContainer: {
    marginBottom: theme.spacing.xl,
  },
  sectionTitle: {
    marginBottom: theme.spacing.sm,
    color: theme.colors.tireMarks,
    fontWeight: 'bold',
  },
  sectionDescription: {
    marginBottom: theme.spacing.lg,
    color: theme.colors.onSurfaceVariant,
  },
  vehicleCard: {
    elevation: 4,
    borderRadius: theme.borderRadius.lg,
    backgroundColor: theme.colors.cardBackground,
    borderWidth: 1,
    borderColor: theme.colors.border,
  },
  vehicleContent: {
    padding: theme.spacing.lg,
  },
  vehicleHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: theme.spacing.lg,
  },
  vehicleIcon: {
    marginRight: theme.spacing.md,
  },
  vehicleInfo: {
    flex: 1,
  },
  vehicleTitle: {
    color: theme.colors.winningRed,
    fontWeight: 'bold',
    marginBottom: theme.spacing.xs,
  },
  vehicleSubtitle: {
    color: theme.colors.onSurfaceVariant,
  },
  vehicleFeatures: {
    marginBottom: theme.spacing.lg,
    paddingLeft: theme.spacing.md,
  },
  featureRow: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: theme.spacing.xs,
  },
  featureBullet: {
    color: theme.colors.winningRed,
    fontSize: 16,
    fontWeight: 'bold',
    marginRight: theme.spacing.sm,
  },
  featureText: {
    color: theme.colors.onSurfaceVariant,
  },
  selectButton: {
    borderRadius: theme.borderRadius.md,
  },
  buttonContent: {
    paddingVertical: theme.spacing.sm,
  },
  quickAccessGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    justifyContent: 'space-between',
  },
  quickAccessCard: {
    width: (width - theme.spacing.lg * 2 - theme.spacing.md) / 2,
    marginBottom: theme.spacing.md,
    elevation: 2,
    borderRadius: theme.borderRadius.md,
    backgroundColor: theme.colors.cardBackground,
    borderWidth: 1,
    borderColor: theme.colors.border,
  },
  quickAccessContent: {
    alignItems: 'center',
    paddingVertical: theme.spacing.lg,
  },
  quickAccessTitle: {
    color: theme.colors.tireMarks,
    fontWeight: 'bold',
    marginTop: theme.spacing.xs,
  },
  quickAccessDesc: {
    color: theme.colors.onSurfaceVariant,
    textAlign: 'center',
    marginTop: theme.spacing.xs,
  },
  statusInfo: {
    backgroundColor: theme.colors.surfaceVariant,
    padding: theme.spacing.md,
    borderRadius: theme.borderRadius.sm,
    marginTop: theme.spacing.lg,
  },
  statusRow: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  statusText: {
    flex: 1,
    color: theme.colors.tireMarks,
    marginLeft: theme.spacing.sm,
  },
});

export default HomeScreen;
