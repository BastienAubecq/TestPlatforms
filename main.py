from game_manager import TextGameManager
from AlphaBetaAgent import AlphaBetaAgent
import CompatibleCAgent

a = AlphaBetaAgent(-1)
b = CompatibleCAgent.CAgent(1)
game_manager = TextGameManager(b, a, 10)

game_manager.play()