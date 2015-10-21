#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Some support tools for evaluating Inverted Pendulum experiments.
"""
import time
import smtplib
from email.mime.text import MIMEText

try:
    import passwords
except:
    print("No passwords file. Won't be able to send e-mail.")


def send_email(subject, body):
    # Try and get sensitive information.
    try:
        sender = passwords.sender
        password = passwords.e_mail_password
        recipients = passwords.recipients
    except Exception as e:
        print(str(e))
        return

    # Connect and authenticate.
    server = smtplib.SMTP('smtp.gmail.com:587')
    #server.set_debuglevel(1)
    server.ehlo()
    server.starttls()
    server.login(sender, password)

    # Form the e-mail.
    msg = MIMEText(body)
    msg['Subject'] = subject
    msg['From'] = sender
    msg['To'] = ", ".join(recipients)

    # Send the message.
    server.sendmail(sender, recipients, msg.as_string())
    server.quit()


def time_elapsed_str(delta):
    """Take the seconds and return a string."""
    ret_str = ""
    days = int(delta / 86400)
    if days:
        ret_str += "%d days " % days
    hours = int((delta % 86400) / 3600)
    if hours or days:
        ret_str += "%d hours " % hours
    minutes = int((delta % 3600) / 60)
    if minutes or days or hours:
        ret_str += "%d minutes " % minutes
    seconds = int(delta % 60)
    ret_str += "%d seconds (%f)" % (seconds, delta)

    return ret_str


def timeit(func=None):
    """Decorator for timing a function and printing runtime to stdout."""
    def inner(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        stop = time.time()

        message = "%s has completed in " % func.__name__
        delta = stop - start
        message += time_elapsed_str(delta)

        print(message)

        return result
    return inner


def reportit(func=None):
    """Decorator for timing a function and sending an e-mail runtime data."""
    def inner(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        stop = time.time()

        message = "%s has completed in " % func.__name__
        delta = stop - start
        days = int(delta / 86400)
        if days:
            message += "%d days " % days
        hours = int((delta % 86400) / 3600)
        if hours or days:
            message += "%d hours " % hours
        minutes = int((delta % 3600) / 60)
        if minutes or days or hours:
            message += "%d minutes " % minutes
        seconds = int(delta % 60)
        message += "%d seconds (%d)" % (seconds, delta)

        send_email("Completion Report - %s" % func.__name__, message)

        return result
    return inner

