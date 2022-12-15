from flask import Flask

def create_app():
    # init app
    server_app = Flask(__name__)

    # Note: This app has 2 blueprints
    # 1. blueprint - app:
    # This blueprient is used as level for basic control app as control,
    # acces to database by read QR code etc ...
    # script -> app routes
    from .routes import app

    server_app.register_blueprint(app)
    return server_app