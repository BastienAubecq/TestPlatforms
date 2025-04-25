from game_manager import TextGameManager
from random_agent import RandomAgent
import CompatibleCAgent

a = RandomAgent(-1)
b = CompatibleCAgent.CAgent(1)
game_manager = TextGameManager(b, a, 10)

game_manager.play()