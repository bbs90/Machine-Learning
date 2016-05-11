import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.q_init(0.5, 0.2, 4)
        #print self.q_val
      
    def q_init(self, alpha, gamma, val):    
        self.q_val = {}
        self.alpha = alpha
        self.gamma = gamma
        for l in ['red', 'green']:
            for d in ['forward', 'left', 'right']:
                for a in [None, 'forward', 'left', 'right']:
                    self.q_val[((d, l), a)] = val

      
    def argmax(self, state):
        q_max = 0.
        for a in [None, 'forward', 'left', 'right']:
            if q_max < self.q_val[(state, a)]:
                q_max = self.q_val[(state, a)]
                a_prime = a
        return q_max, a_prime


    def q_function(self, state, reward):
        q, action = self.argmax(self.state)
        (q_prime, action_prime) = self.argmax(state)        
        q =(1 - self.alpha) * q + (self.alpha * (reward + self.gamma * q_prime)) 
        return q

        
    def q_update(self, q, action, reward):
        next_waypoint = self.planner.next_waypoint()
        inputs = self.env.sense(self)
        s_prime = (next_waypoint, inputs['light']) 
        self.q_val[(self.state, action)] = self.q_function(s_prime, reward)

    
    def reset(self, destination=None):
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required


    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)

        # TODO: Update state
        #print inputs
        self.state = self.next_waypoint, inputs['light']
        
        
        
        # TODO: Select action according to your policy
        """
        action = None
        actions = [None, 'forward', 'left', 'right']
        
        #if self.state[1] == 'green':
			#action = random.choice(actions)
		#action = random.choice(actions)
        """
		
        q, action = self.argmax(self.state)
        # Execute action and get reward
        reward = self.env.act(self, action)

        # TODO: Learn policy based on state, action, reward
        self.q_update(q, action,reward)
       
        print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # set agent to track

    # Now simulate it
    sim = Simulator(e, update_delay=0)  # reduce update_delay to speed up simulation
    sim.run(n_trials=100)  # press Esc or close pygame window to quit


if __name__ == '__main__':
    run()
