from flask import Blueprint, render_template

# set 1. blueprint = app
app = Blueprint("app", __name__)

@app.route("/")
def robot_connect():
    return render_template("robot_connect.html")

@app.route("/robot_control")
def robot_control():
    return render_template("robot_control.html")