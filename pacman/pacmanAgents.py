# pacmanAgents.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from pacman import Directions
from game import Agent
from heuristics import *
import random

class RandomAgent(Agent):
	# Initialization Function: Called one time when the game starts
	def registerInitialState(self, state):
		return;

	# GetAction Function: Called with every frame
	def getAction(self, state):
		# get all legal actions for pacman
		actions = state.getLegalPacmanActions()
		# returns random action from all the valide actions
		return actions[random.randint(0,len(actions)-1)]

class OneStepLookAheadAgent(Agent):
	# Initialization Function: Called one time when the game starts
	def registerInitialState(self, state):
		return;

	# GetAction Function: Called with every frame
	def getAction(self, state):
		# get all legal actions for pacman
		legal = state.getLegalPacmanActions()
		# get all the successor state for these actions
		successors = [(state.generatePacmanSuccessor(action), action) for action in legal]
		# evaluate the successor states using scoreEvaluation heuristic
		scored = [(admissibleHeuristic(state), action) for state, action in successors]
		# get best choice
		bestScore = min(scored)[0]
		# get all actions that lead to the highest score
		bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
		# return random action from the list of the best actions
		return random.choice(bestActions)

class BFSAgent(Agent):
	queue = None
	S = None
	c = None
	path = None

	# Initialization Function: Called one time when the game starts
	def registerInitialState(self, state):
		return;
	
	# GetAction Function: Called with every frame
	def getAction(self, state):
		pathofpacman = []
		queue = []
		succ = []
		cost = 0
		cost_arr = []
		parent_depth = 1
		depth = 0
		visited = set()

		init_legal = state.getLegalPacmanActions()
		queue = [(state.generatePacmanSuccessor(action), action, parent_depth + admissibleHeuristic(state.generatePacmanSuccessor(action)), parent_depth) for action in init_legal]
		cost_arr = [(node[2], node[1]) for node in queue]
		
		while queue:
			vertex, actions, cost, depth = queue[0]
			del queue[0]
			
			if vertex.isWin():
				return random.choice(actions)
				
			if(vertex == None):
				break
			if(vertex not in visited):
				visited.add(vertex)
				legal = vertex.getLegalPacmanActions()
				for action in legal:
					child = vertex.generatePacmanSuccessor(action)
					if child is None:
						break
					depth = parent_depth + 1
					cost = admissibleHeuristic(child)
					queue.append((child, action, cost, depth))
					cost_arr.append((cost, action))
			
			if(queue):
				cost_arr.sort(key = lambda x:x[0])
				for node in cost_arr:
					pathofpacman.append(node[1])
		return random.choice(pathofpacman)
				
class DFSAgent(Agent):
	stack = None
	H = None
	c = None
	path = None

	# Initialization Function: Called one time when the game starts
	def registerInitialState(self, state):
		return;

	# GetAction Function: Called with every frame
	def getAction(self, state):
		# TODO: write DFS Algorithm instead of returning Directions.STOP
		stack = []
		path=[]
		H = []
		visited = []
		c = 0
		pathofpacman = []
		succ = []
		cost = 0
		cost_arr = []
		parent_depth = 1
		depth = 0
		
		stack.append((state,[]))
		H.append(admissibleHeuristic(state))
		
		while stack:
			vertex, path = stack.pop()
			current_heuristic = H.pop()

			if vertex.isWin():
				return action

			if vertex not in visited:
				visited.append(vertex)
				legal = vertex.getLegalPacmanActions()
				succ = [(vertex.generatePacmanSuccessor(action)) for action in legal]
				for i in succ:
					if i not in visited:
						H.append(admissibleHeuristic(i))        
				m = H[0]
				for i in H:
					if m > i:
						m = i
				m_index = H.index(m)

				for i in succ:
					if i not in visited:                      
						if succ.index(i) == m_index:
							path = path + [legal[m_index]]
							pathofpacman.append(path)
							stack.append(( i, path))
							H.append(admissibleHeuristic(i))

		action = path[c]
		c = c + 1
		return action

class AStarAgent(Agent):
	# Initialization Function: Called one time when the game starts
	def registerInitialState(self, state):
		return;

	# GetAction Function: Called with every frame
	def getAction(self, state):
		# TODO: write A* Algorithm instead of returning Directions.STOP
		queue = []
		cost_arr = []
		cost = 0
		depth = 0
		parent_depth = 1

		init_legal = state.getLegalPacmanActions()
		queue = [(state.generatePacmanSuccessor(action), action, parent_depth + admissibleHeuristic(state.generatePacmanSuccessor(action)), parent_depth) for action in init_legal]
		cost_arr = [(node[2], node[1]) for node in queue]
		
		while queue:
			queue.sort(key = lambda x:x[2])
			vertex, actions, cost, depth = queue[0]
			del queue[0]
			
			if vertex.isWin():
				return random.choice(actions)
				
			if(vertex == None):
				break
			legal = vertex.getLegalPacmanActions()
			for action in legal:
				child = vertex.generatePacmanSuccessor(action)
				if child is None:
					break
				depth = parent_depth + 1
				cost = depth + admissibleHeuristic(child)
				queue.append((child, action, cost, depth))
				cost_arr.append((cost, action))
			
			if(queue):
				cost_arr.sort(key = lambda x:x[0])
				bestScore, bestActions = cost_arr.pop()
		return bestActions