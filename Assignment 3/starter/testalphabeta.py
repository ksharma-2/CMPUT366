import random
import unittest 
from connect4 import Board
from math import inf

def alpha_beta(board, player, alpha, beta, ply):
	"""
	Function receives an instances of the Board class, the player who is to act at this state (either X or O),
	the value of alpha, beta, and the maximum search depth given by the variable ply.

	The function returns three values: 
	1. the score of the optimal move for the player who is to act;
	2. the optimal move
	3. the total number of nodes expanded to find the optimal move 
	"""
	if board.is_terminal():
		return board.game_value(), 0, 1
	
	if ply == 0:
		return 0, 0, 1

	expansions = 1
	newMove = 0
	bestMove = 0

	# print(board)

	
	if player == 'X':
		bestScore = float('-inf')
		for x in board.available_moves():

			board.perform_move(x, player)
			# print(board)
			testScore, testMove, testExpansion = alpha_beta(board, 'O', alpha, beta, ply-1)
			newMove += 1
			expansions += testExpansion
			board.undo_move(x)

			if testScore > bestScore:
				bestScore = testScore
				bestMove = newMove - 1

			if bestScore >= beta:
				return bestScore, bestMove, expansions
			alpha = max(alpha, bestScore)

		return bestScore, bestMove, expansions
	else:
		bestScore = float('inf')
		for x in board.available_moves():

			board.perform_move(x, player)
			testScore, testMove, testExpansion = alpha_beta(board, 'X', alpha, beta, ply-1)
			newMove += 1

			if testScore < bestScore:
				bestScore = testScore
				bestMove = newMove - 1

			expansions += testExpansion

			board.undo_move(x)

			if bestScore <= alpha:
				return bestScore, bestMove, expansions
			beta = min(beta, bestScore)

		return bestScore, bestMove, expansions


class TestMinMaxDepth1(unittest.TestCase):

	def test_depth1a(self):
		b = Board()
		player = b.create_board('010101')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 0)

	def test_depth1b(self): 
		b = Board() 
		player = b.create_board('001122')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 3)

	def test_depth1c(self): 
		b = Board() 
		player = b.create_board('335566')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 4)

	def test_depth1d(self):
		b = Board() 
		player = b.create_board('3445655606')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 6)

	def test_depth1e(self):
		b = Board() 
		player = b.create_board('34232210101')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 1)

	def test_depth1f(self):
		b = Board() 
		player = b.create_board('23445655606')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 6)

	def test_depth1g(self): 
		b = Board() 
		player = b.create_board('33425614156')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 1)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 2)

class TestMinMaxDepth3(unittest.TestCase):

	def test_depth3a(self):
		b = Board()
		player = b.create_board('303111426551')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 2)

	def test_depth3b(self): 
		b = Board() 
		player = b.create_board('23343566520605001')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 6)

	def test_depth3c(self): 
		b = Board() 
		player = b.create_board('10322104046663')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 0)

	def test_depth3d(self):
		b = Board() 
		player = b.create_board('00224460026466')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 3)

	def test_depth3e(self):
		b = Board() 
		player = b.create_board('102455500041526')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 1)

	def test_depth3f(self):
		b = Board() 
		player = b.create_board('01114253335255')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 2)

	def test_depth3g(self): 
		b = Board() 
		player = b.create_board('0325450636643')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 3)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 5)

class TestMinMaxDepth5(unittest.TestCase):
	def test_depth5a(self):
		b = Board()
		player = b.create_board('430265511116')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 3)
		
	def test_depth5b(self):
		b = Board()
		player = b.create_board('536432111330')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 5)

	def test_depth5c(self):
		b = Board()
		player = b.create_board('322411004326')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 3)

	def test_depth5d(self):
		b = Board()
		player = b.create_board('3541226000220')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 4)

	def test_depth5e(self):
		b = Board()
		player = b.create_board('43231033655')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		self.assertEqual(bestScore, -1)
		self.assertEqual(bestMove, 1)

	def test_depth5f(self):
		b = Board()
		player = b.create_board('345641411335')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 5)

	def test_depth5g(self):
		b = Board()
		player = b.create_board('336604464463')
		bestScore, bestMove, expansions = alpha_beta(b, player, -inf, inf, 5)
		# print(b)

		self.assertEqual(bestScore, 1)
		self.assertEqual(bestMove, 3)
		print("\nExpansions: " + str(expansions))

if __name__ == '__main__':
    # b = Board()
    # Board.host_game(b)
    unittest.main()