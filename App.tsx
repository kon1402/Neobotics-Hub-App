import React from 'react';
import {NavigationContainer} from '@react-navigation/native';
import {createStackNavigator} from '@react-navigation/stack';
import {Provider as PaperProvider} from 'react-native-paper';
import {SafeAreaProvider} from 'react-native-safe-area-context';
import Toast from 'react-native-toast-message';

import {theme} from './src/theme/theme';
import HomeScreen from './src/screens/HomeScreen';
import TeamSelectionScreen from './src/screens/TeamSelectionScreen';
import CarControlScreen from './src/screens/CarControlScreen';

export type RootStackParamList = {
  Home: undefined;
  TeamSelection: undefined;
  CarControl: {teamNumber: number; ipAddress: string};
};

const Stack = createStackNavigator<RootStackParamList>();

function App(): JSX.Element {
  return (
    <SafeAreaProvider>
      <PaperProvider theme={theme}>
        <NavigationContainer>
          <Stack.Navigator
            initialRouteName="Home"
            screenOptions={{
              headerStyle: {
                backgroundColor: theme.colors.tireMarks,
              },
              headerTintColor: theme.colors.winningRed,
              headerTitleStyle: {
                fontWeight: 'bold',
                fontSize: 18,
                color: theme.colors.winningRed,
              },
            }}>
            <Stack.Screen
              name="Home"
              component={HomeScreen}
              options={{title: 'Neobotics Car Control'}}
            />
            <Stack.Screen
              name="TeamSelection"
              component={TeamSelectionScreen}
              options={{title: 'Select Team'}}
            />
            <Stack.Screen
              name="CarControl"
              component={CarControlScreen}
              options={({route}) => ({
                title: `Team ${route.params.teamNumber} Control`,
              })}
            />
          </Stack.Navigator>
        </NavigationContainer>
        <Toast />
      </PaperProvider>
    </SafeAreaProvider>
  );
}

export default App;
