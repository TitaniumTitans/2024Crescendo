from __future__ import annotations
import typing
import typing_extensions
__all__ = ['Client', 'ClientPublisher', 'ClientSubscriber', 'SubscriberOptions', 'TopicPublisher', 'TopicSubscriber', 'decodeClientPublishers', 'decodeClientSubscribers', 'decodeClients', 'decodeTopicPublishers', 'decodeTopicSubscribers']
class Client:
    """
    Client (as published via `$clients`).
    """
    conn: str
    id: str
    version: int
    def __init__(self) -> None:
        ...
class ClientPublisher:
    """
    Client publisher (as published via `$clientpub$<client>` or `$serverpub`).
    """
    topic: str
    uid: int
    def __init__(self) -> None:
        ...
class ClientSubscriber:
    """
    Client subscriber (as published via `$clientsub$<client>` or `$serversub`).
    """
    options: SubscriberOptions
    topics: list[str]
    uid: int
    def __init__(self) -> None:
        ...
class SubscriberOptions:
    """
    Subscriber options. Different from PubSubOptions in this reflects only
    options that are sent over the network.
    """
    periodic: float
    prefixMatch: bool
    sendAll: bool
    topicsOnly: bool
    def __init__(self) -> None:
        ...
class TopicPublisher:
    """
    Topic publisher (as published via `$pub$<topic>`).
    """
    client: str
    pubuid: int
    def __init__(self) -> None:
        ...
class TopicSubscriber:
    """
    Topic subscriber (as published via `$sub$<topic>`).
    """
    client: str
    options: SubscriberOptions
    subuid: int
    def __init__(self) -> None:
        ...
def decodeClientPublishers(data: typing_extensions.Buffer) -> list[ClientPublisher] | None:
    """
    Decodes `$clientpub$<topic>` meta-topic data.
    
    :param data: data contents
    
    :returns: Vector of ClientPublishers, or empty optional on decoding error.
    """
def decodeClientSubscribers(data: typing_extensions.Buffer) -> list[ClientSubscriber] | None:
    """
    Decodes `$clientsub$<topic>` meta-topic data.
    
    :param data: data contents
    
    :returns: Vector of ClientSubscribers, or empty optional on decoding error.
    """
def decodeClients(data: typing_extensions.Buffer) -> list[Client] | None:
    """
    Decodes `$clients` meta-topic data.
    
    :param data: data contents
    
    :returns: Vector of Clients, or empty optional on decoding error.
    """
def decodeTopicPublishers(data: typing_extensions.Buffer) -> list[TopicPublisher] | None:
    """
    Decodes `$pub$<topic>` meta-topic data.
    
    :param data: data contents
    
    :returns: Vector of TopicPublishers, or empty optional on decoding error.
    """
def decodeTopicSubscribers(data: typing_extensions.Buffer) -> list[TopicSubscriber] | None:
    """
    Decodes `$sub$<topic>` meta-topic data.
    
    :param data: data contents
    
    :returns: Vector of TopicSubscribers, or empty optional on decoding error.
    """
