from threading import Lock


class BaseAction:
    lock = Lock()
