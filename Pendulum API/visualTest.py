import kivy
kivy.require('1.7.2')

from pendulum import Pendulum
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput

class LoginScreen(GridLayout):
	def __init__(self, **kwargs):
		super(LoginScreen, self).__init__(**kwargs)
		self.cols = 2
		self.add_widget(Label(text='User Name'))
		self.username = TextInput(multiline=False)
		self.add_widget(self.username)
		self.add_widget(Label(text='password'))
		self.add_widget(TextInput(password=True, multiline=False))

class MyApp(App):
	def build(self):
		return LoginScreen()
if __name__ == '__main__':
	MyApp().run()
