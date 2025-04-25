from time import time
from fenix import *
from agent import Agent

class AlphaBetaAgent(Agent):

    def __init__(self, player):
        super().__init__(player)
        self.explored = 0
        self.hit = 0

    def act(self, state: FenixState, remaining_time: float) -> FenixAction:
        allocatedTime = remaining_time / 20
        self.evaluationUsed = False # Wether or not evaluation function has been used to compute the previous score
        return self.iterativeDeepening(state, allocatedTime)
    

    # Search the tree progressively deeper for a given ammount of time
    def iterativeDeepening(self, state: FenixState, maxTime: float) -> FenixAction:
        depth = 0
        actions = state.actions()
        bestAction = None
        timeFactors = [] # Time search factors for increasing depth
        previousTime = None
        while len(timeFactors) == 0 or previousTime * sum(timeFactors) / len(timeFactors) < maxTime:
            #if len(timeFactors) != 0:
            #    print(f"Expected time is ok: {previousTime * sum(timeFactors) / len(timeFactors)}")
            #print(f"Starting depth: {depth+1}")
            tStart = time()

            alpha = -1.1
            self.evaluationUsed = False
            for action in actions:
                newState = state.result(action)
                if newState.is_terminal():
                    score = newState.utility(state.current_player)
                else:
                    score = -self.negamax(newState, depth, -1, -alpha)
                if score == 1: # Win :)
                    #print("Win found")
                    return action
                if score > alpha:
                    alpha = score
                    bestAction = action
            if not self.evaluationUsed:
                #print("Evaluation function not used, result can be trusted")
                return bestAction

            # Predict if we have time to do one more iteration
            newTime = time() - tStart
            maxTime -= newTime
            #print(f"Remaining time: {maxTime}")
            if previousTime is not None:
                timeFactors.append(newTime / previousTime)
            previousTime = newTime
            depth += 1
        #print(f"Expected time is too much: {previousTime * sum(timeFactors) / len(timeFactors)}")
        return bestAction
    

    # Min and max minimax functions at the same time (alpha, beta and score are reversed to switch between min and max)
    def negamax(self, state: FenixState, depth: int, alpha: float, beta: float) -> float:
        if depth == 0: # Stop search -> evaluation function
            return self.score(state)
        
        # Check transposition table
        """key = state._hash()
        item = table.get(key, depth)
        self.explored += 1
        if item is not None:
            self.hit += 1
            if item.lowerBound and item.value > alpha:
                alpha = item.value
            if item.upperBound and item.value < beta:
                beta = item.value
            if alpha >= beta:
                return alpha"""
        
        # Find best action
        actions = state.actions()
        found = False
        for action in actions:
            newState = state.result(action)
            if newState.is_terminal():
                score = newState.utility(state.current_player)
            else:
                score = -self.negamax(newState, depth - 1, -beta, -alpha)
            if score >= beta:
                #table.put(key, score, depth, False, True)
                return score
            if score > alpha:
                alpha = score
                found = True

        #table.put(key, alpha, depth, True, found)
        return alpha


    # Quickly evaluate a state
    def score(self, state: FenixState) -> float:
        self.evaluationUsed = True
        currentPieces = 0
        otherPieces = 0
        for i in range(state.dim[0]):
            for j in range(state.dim[1]):
                if (i,j) in state.pieces:
                    n = state.pieces[(i,j)]
                    if n * state.current_player > 0: currentPieces += abs(n)
                    else: otherPieces += abs(n)
        score = (currentPieces - otherPieces) / (currentPieces + otherPieces)
        if score == 0: # Prefer less pieces to avoid draws
            score += (42 - currentPieces - otherPieces) / 2000
        return score
    

class TranspositionTable:
    class Item:
        def __init__(self, key: int, value: float, depth: int, upperBound: bool, lowerBound: bool):
            self.key = key
            self.value = value
            self.depth = depth
            self.upperBound = upperBound
            self.lowerBound = lowerBound

    def __init__(self):
        self.size = 34000001
        self.table = [None for _ in range(self.size)]

    def get(self, key: int, depth: int) -> Item | None:
        item = self.table[key % self.size]
        if item is not None and item.key == key and depth <= item.depth:
            return item
        return None
    
    def put(self, key: int, value: float, depth: int, upperBound: bool, lowerBound: bool):
        item = self.table[key % self.size]
        if item is None or item.key != key:
            self.table[key % self.size] = self.Item(key, depth, value, upperBound, lowerBound)
        elif depth > item.depth:
            item.value = value
            item.depth = depth
            item.upperBound = upperBound
            item.lowerBound = lowerBound

#table = TranspositionTable()