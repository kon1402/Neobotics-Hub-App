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
              source={require('../assets/images/Neobotics_Primary_WTransparent.png')}
              style={styles.logoImage}
              resizeMode="contain"
            />
            <Text variant="titleLarge" style={styles.subtitle}>
              Control Hub
            </Text>
          </View>
        </Surface>

        {/* Main Hub Content */}
        <View style={styles.content}>
          {/* Platform Selection Section */}
          <View style={styles.sectionContainer}>
            <Text variant="titleLarge" style={styles.sectionTitle}>
              Select Platform
            </Text>

            {/* NeoRacer Platform Card */}
            <Card style={styles.platformCard} onPress={handleSelectVehicle}>
              <Card.Content style={styles.platformContent}>
                {/* Car Image Placeholder */}
                <View style={styles.carImageContainer}>
                  <View style={styles.carImagePlaceholder}>
                    <IconButton
                      icon="car-sports"
                      size={60}
                      iconColor={theme.colors.winningRed}
                    />
                    <Text variant="bodySmall" style={styles.placeholderText}>
                      NeoRacer Image
                    </Text>
                  </View>
                </View>

                {/* NeoRacer Title */}
                <Text style={styles.neoRacerTitle}>
                  NeoRacer
                </Text>

                {/* Select Button */}
                <Button
                  mode="contained"
                  onPress={handleSelectVehicle}
                  style={styles.selectButton}
                  contentStyle={styles.buttonContent}
                  buttonColor={theme.colors.winningRed}
                  icon="arrow-right">
                  Select Platform
                </Button>
              </Card.Content>
            </Card>
          </View>
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
    paddingVertical: 4,
    marginBottom: 4,
  },
  headerContent: {
    alignItems: 'center',
  },
  logoImage: {
    width: 700,
    height: 350,
    marginBottom: 0,
  },
  subtitle: {
    color: theme.colors.circuitSteel,
    fontWeight: '600',
  },
  content: {
    paddingHorizontal: theme.spacing.lg,
    paddingBottom: theme.spacing.xl,
  },
  sectionContainer: {
    marginBottom: theme.spacing.xl,
  },
  sectionTitle: {
    marginBottom: theme.spacing.lg,
    color: theme.colors.tireMarks,
    fontWeight: 'bold',
    textAlign: 'center',
  },
  platformCard: {
    elevation: 4,
    borderRadius: theme.borderRadius.lg,
    backgroundColor: theme.colors.cardBackground,
    borderWidth: 1,
    borderColor: theme.colors.border,
  },
  platformContent: {
    padding: theme.spacing.xl,
    alignItems: 'center',
  },
  carImageContainer: {
    marginBottom: theme.spacing.xl,
  },
  carImagePlaceholder: {
    width: 200,
    height: 120,
    backgroundColor: theme.colors.surfaceVariant,
    borderRadius: theme.borderRadius.md,
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 2,
    borderColor: theme.colors.border,
    borderStyle: 'dashed',
  },
  placeholderText: {
    color: theme.colors.onSurfaceVariant,
    marginTop: theme.spacing.xs,
    textAlign: 'center',
  },
  neoRacerTitle: {
    fontSize: 32,
    fontFamily: 'System', // Will update this when you provide the Apotek Extended Bold font
    fontWeight: 'bold',
    color: theme.colors.winningRed,
    marginBottom: theme.spacing.xl,
    textAlign: 'center',
    letterSpacing: 1,
  },
  selectButton: {
    borderRadius: theme.borderRadius.md,
    minWidth: 200,
  },
  buttonContent: {
    paddingVertical: theme.spacing.md,
  },
});

export default HomeScreen;
