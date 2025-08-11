import {MD3LightTheme} from 'react-native-paper';

// Neobotics official color scheme
export const colors = {
  // Primary brand colors - Neobotics Winning Red
  primary: '#EB341C', // Winning Red
  primaryContainer: '#FFEBE8',
  onPrimary: '#FFFFFF',
  onPrimaryContainer: '#5A0A00',
  
  // Secondary colors - Tire Marks (dark)
  secondary: '#1D1D27', // Tire Marks
  secondaryContainer: '#BFBFBF', // Circuit Steel
  onSecondary: '#FFFFFF',
  onSecondaryContainer: '#1D1D27',
  
  // Neutral colors using Circuit Steel
  surface: '#FFFFFF',
  onSurface: '#1D1D27',
  surfaceVariant: '#F5F5F5',
  onSurfaceVariant: '#6A6A6A',
  
  // Background
  background: '#FAFAFA',
  onBackground: '#1D1D27',
  
  // Neobotics specific colors
  winningRed: '#EB341C',
  tireMarks: '#1D1D27',
  circuitSteel: '#BFBFBF',
  
  // Status colors (adapted to Neobotics palette)
  success: '#4CAF50',
  warning: '#FF9800',
  error: '#EB341C', // Use Neobotics red for errors
  info: '#1D1D27', // Use Tire Marks for info
  
  // Car status indicators
  connected: '#4CAF50',
  disconnected: '#EB341C',
  connecting: '#FF9800',
  
  // Additional UI colors
  border: '#BFBFBF', // Circuit Steel for borders
  shadow: 'rgba(29, 29, 39, 0.15)', // Tire Marks with opacity
  cardBackground: '#FFFFFF',
  accent: '#EB341C',
};

export const theme = {
  ...MD3LightTheme,
  colors: {
    ...MD3LightTheme.colors,
    ...colors,
  },
  fonts: {
    ...MD3LightTheme.fonts,
    // Using system fonts that work well on mobile
    headlineLarge: {
      fontSize: 32,
      fontWeight: '700' as const,
      letterSpacing: 0,
      lineHeight: 40,
    },
    headlineMedium: {
      fontSize: 28,
      fontWeight: '600' as const,
      letterSpacing: 0,
      lineHeight: 36,
    },
    headlineSmall: {
      fontSize: 24,
      fontWeight: '600' as const,
      letterSpacing: 0,
      lineHeight: 32,
    },
    titleLarge: {
      fontSize: 22,
      fontWeight: '500' as const,
      letterSpacing: 0,
      lineHeight: 28,
    },
    titleMedium: {
      fontSize: 16,
      fontWeight: '500' as const,
      letterSpacing: 0.15,
      lineHeight: 24,
    },
    bodyLarge: {
      fontSize: 16,
      fontWeight: '400' as const,
      letterSpacing: 0.5,
      lineHeight: 24,
    },
    bodyMedium: {
      fontSize: 14,
      fontWeight: '400' as const,
      letterSpacing: 0.25,
      lineHeight: 20,
    },
    labelLarge: {
      fontSize: 14,
      fontWeight: '500' as const,
      letterSpacing: 0.1,
      lineHeight: 20,
    },
  },
  spacing: {
    xs: 4,
    sm: 8,
    md: 16,
    lg: 24,
    xl: 32,
    xxl: 48,
  },
  borderRadius: {
    sm: 8,
    md: 12,
    lg: 16,
    xl: 24,
  },
};
