import abc


class PlcDriver(abc.ABC):
    @abc.abstractmethod
    def connect(self): ...
    @abc.abstractmethod
    def read(self, addrs): ...
    @abc.abstractmethod
    def write(self, values): ...
    def set_signal_types(self, types): pass
    def set_signal_lengths(self, lengths): pass
    def set_signal_addressing(self, meta): pass
    def get_server_types(self): return None
    def close(self): pass
