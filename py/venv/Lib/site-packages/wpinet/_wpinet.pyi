from __future__ import annotations
__all__ = ['PortForwarder']
class PortForwarder:
    """
    Forward ports to another host.  This is primarily useful for accessing
    Ethernet-connected devices from a computer tethered to the RoboRIO USB port.
    """
    @staticmethod
    def getInstance() -> PortForwarder:
        """
        Get an instance of the PortForwarder class.
        
        This is a singleton to guarantee that there is only a single instance
        regardless of how many times GetInstance is called.
        """
    def add(self, port: int, remoteHost: str, remotePort: int) -> None:
        """
        Forward a local TCP port to a remote host and port.
        Note that local ports less than 1024 won't work as a normal user.
        
        :param port:       local port number
        :param remoteHost: remote IP address / DNS name
        :param remotePort: remote port number
        """
    def remove(self, port: int) -> None:
        """
        Stop TCP forwarding on a port.
        
        :param port: local port number
        """
