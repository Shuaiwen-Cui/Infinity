"""Proxy for Redis sentinel."""
from redis.sentinel import Sentinel


class SentinelProxy:
    """Proxy for Redis sentinel."""

    def __init__(self, sentinel_host, master_name,
                 socket_timeout=0.1, **kwargs):
        """Initialize Redis sentinel connection.

        :params: sentinel_host: (host, port)
        """
        self.sentinel = Sentinel(
            [sentinel_host], socket_timeout=socket_timeout)
        self.master = self.sentinel.master_for(master_name,
                                               socket_timeout=socket_timeout,
                                               **kwargs)
        self.slave = self.sentinel.slave_for(master_name,
                                             socket_timeout=socket_timeout,
                                             **kwargs)

    def __getattr__(self, name):
        """Get attribute from Redis master or slave."""
        master_key = ('set', 'hset', 'hmset',
                      'lset', 'lpush', 'blpop',
                      'brpop', 'rpush', 'expire',
                      'delete', 'incr')
        if name not in master_key:
            target = self.slave
        else:
            target = self.master
        return getattr(target, name)
