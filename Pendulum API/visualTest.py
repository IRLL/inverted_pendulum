import kivy
kivy.require('1.7.2')

from pendulum import Pendulum
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput

class LoginScreen(GridLayout):
	def __init__(self, **kwargs):
		self.p = Pendulum('/dev/ACMtty0', '/dev/ACMtty1')
		super(LoginScreen, self).__init__(**kwargs)
		self.cols = 2
		self.add_widget(Label(text='Bytes: '))
		self.add_widget(Label(text=self.p.uc.getVariables()))

class MyApp(App):
	def build(self):
		return LoginScreen()
if __name__ == '__main__':
	MyApp().run()
