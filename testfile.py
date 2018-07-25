
class App:
	def __init__(self):
		self.a = 1
		self.b = 2
		self.thread = Thread(self)

	def get_a_from_thread(self):
		self.thread.get_a()


class Thread:
	def __init__(self, app):
		self.app = app

	def get_a(self):
		print self.app.a


if __name__ == "__main__":
	a = App()
	a.get_a_from_thread()