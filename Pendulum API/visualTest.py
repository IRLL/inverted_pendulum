import kivy
kivy.require('1.7.2')

from pendulum import Pendulum
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput

class LoginScreen(GridLayout):
	i = StringProperty('')
	def __init__(self, *args, **kwargs):
		super(LoginScreen, self).__init__(*args, **kwargs)
		self.cols = 2
		self.add_widget(Label(text='Bytes: '))
		self.bind(i=self.incI())
		self.add_widget(Label(text=self.i))
	def incI(self, instance)
		self.i = self.i + 1

class MyApp(App):
	def build(self):
		return LoginScreen()
if __name__ == '__main__':
	MyApp().run()
