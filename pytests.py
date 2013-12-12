
def outer():
	a = 'a'

	def inner():
		def inner2():
			print(a)

		inner2()

	inner()
class A:

	class B:
		def __init__(self):
			print('inside B!')
			print(A.self)

	def __init__(self):
		self.b = self.B()


if __name__ == '__main__':
	outer()