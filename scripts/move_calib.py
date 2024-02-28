from ambf_client import Client

# Generate all objects in scene, even ones that may not be needed
_client = Client()
_client.connect()
print(_client.get_obj_names())


