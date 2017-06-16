/******************** agent.h_07Mar2017 ********************
 *  Created 07 March, 2017
 *  Author: Nick Livingston (nilivingston@vassar.edu)
 *  
 *  This header file is the file that will be altered by
 *  the L16A genotype to phenotype (G to P) mapping process.
 *  It contains information about which sensors are in use,
 *  how many sensors there are, which motors are in use (of
 *  the two available iRobot Create 2 motors), and how many
 *  neurons there are at the input, hidden, and output
 *  layers.  The input neurons will represent the sensors in
 *  use, and the output neurons will represent the motors in
 *  use.  
************************************************************/
#define NUM_INPUT  1
#define NUM_HIDDEN 1
#define NUM_OUTPUT 2

/* Array specifying which sensors are being used for input.
 */
int port[NUM_INPUT] = {A1};

/* Arrays for the agent's input, hidden neuron, and output
 * neuron values.  The input will be updated by sensor  
 * readings.  The hidden values will be updated by the input 
 * values, and then by old hidden values (through recurrent
 * connections).  The old hidden values will be updated by
 * the fully updated current hidden values.  And the output
 * values will be updated by the current hidden values.
 */
int input[NUM_INPUT];
int hidden[NUM_HIDDEN];
int old_hidden[NUM_HIDDEN];
int output[NUM_OUTPUT];

/* Neural network connection weights.  These are the
 * weights that will be used to determine how each of
 * the "value" arrays updates the others.
 */
int input_to_hidden[NUM_INPUT][NUM_HIDDEN] = {{1}};
int hidden_to_hidden[NUM_HIDDEN][NUM_HIDDEN] = {{0}};
int hidden_to_output[NUM_HIDDEN][NUM_OUTPUT] = {{1, -1}};

/*
If non-recurrent layer:
y_i^(t) = tanh( y_i^(t-1)  + tau_i * ( sum_j  w_ji * y_j^( t )   ) )
Otherwise (if i and j are both hidden recurrent neurons),
y_i^(t) = tanh( y_i^(t-1)  + tau_i * ( sum_j  w_ji * y_j^( t-1 )   ) )

Tau_i = [-1.0 , +1.0]
*/
