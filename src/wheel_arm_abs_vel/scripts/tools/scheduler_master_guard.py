import xmlrpc.client
class BoundedTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 1.0
        return connection
def master_pid(uri):
    try:
        with xmlrpc.client.ServerProxy(uri, transport=BoundedTransport()) as master:
            result = master.getPid("/kuavo_observer_guard")
    except (xmlrpc.client.Error, ValueError) as error:
        raise RuntimeError("master query failed") from error
    if result[0] != 1:
        raise RuntimeError("master unavailable")
    return result[2]
