# library -> flask
from flask import Blueprint, render_template

# set 1. blueprint = app
app = Blueprint("app", __name__)

# page -> route /:
# Note:
#   First(Main) page with robot connect
@app.route("/")
def robot_connect():
    return render_template("robot_connect.html")

# page -> route robot_control:
# Note:
#   Second page robot control page
@app.route("/robot_control")
def robot_control():
    return render_template("robot_control.html")