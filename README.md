# Mappd - Modular applications service
## Short overview
The main goal of the project is to create an architecture that simplifies the development of complex network applications. Some ideas were taken from AUTOSAR, mDNS and others. Read below how it works.
## How it works
### Networking

![](image_url)

Let's consider this case. Several nodes connected to some local network it may be VPN or Mesh. Each node has local IP address and connected by "each to each" scheme with others.
Let's explain more detailed:

![](image_url)

Network has one or two (or more) multicast points -- several ip addresses used as "directory". Each node able to use one or more multicast points. These are serve to discover other "peer" environments which will be explained later. UDP protocol is used for multicasting and receiveing data. All traffic is encrypted by AES. To discover ip addresses and ports that will be used in future TLS connections each node must to receive and decrypt these messages. When environments are discovered and their credentials are cached, node follows to creating of secure connections with others. During TLS handshake we use generated certificate-key pairs and local storaged CA keypair. This pair used for signing of generated pairs and verifying of peer certificates. Meanwhile, user code starts executing. Let's consider mappd workflow:

### Mappd workflow
1. Parsing .conf file;
Actually there may be several of these files. But there shall be one main config, others are included. This point is done when content of .conf is represented as mappd config structure in process memory. As example of config file:

```conf
environment {
	name: "test_env"
	type: "static"
	domain_networking_enabled : false
	headers : "/usr/local/include/mappd/include" 	# This directory contains c++ headers to be used in building of architecture entities.
	ca_certificate : "/usr/local/share/mappd/ca/ca_cert.crt"	# Root CA certificate (encrypted)
	ca_key : "/usr/local/share/mappd/ca/ca_key.key"	# Root CA key (encrypted)
	aes_passphrase : "/usr/local/share/mappd/pass/pass.txt" # AES passphrase, encrypted by yourself, used for receiveing of multicasts, decrypting of CA files
	fuse_mountpoint : "/var/run/mappd"	# Environment filesystem mountpoint
	ca_fs_mountpoint : "/var/run/mappd/ca_files"	# CA files in FUSE location
	arch_fs_mountpoint : "/var/run/mappd/arch"	# Mountpoint for architecture content (for administration)
	...
	network_conf : {
		interface_name : "enp1s0"
		ipv6_enabled : true
		mcast_grps : [{"224.0.0.1", 4444}, {"224.0.0.2", 4445}, {"ff00::/8", 4446}]
		...
	},

	root_composition : {
		name : "test_composition_root"
		ports: [port {
			name : "root_composition_port_test"
			port_type: "sender"									# Sender, Receiver, Client or Server
			port_data_type: "int32_t"						#  Any known data type
		}, ... ],

		compositions : { ... }
		components : { ... }
	}
}

...
```
2. When, building of environments, based of parsed config. Just load if it already build. There are two ways:
	- Static environment. 
Built as .so object, but is not modifiable. Can be loaded or unloaded only.

	- Dynamic environment. 
Each entity, such as compositions, ports, components can be loaded/unloaded/replaced in runtime.

3. Store built environment in directory, specified in config file.
4. Load and run.
5. Use inotify to detect if config file changed.
6. Goto point 1.

![](image_url)

### Environment
- Root composition
Main component of environment arch. Contains all user defined components and compositions.

- Root ports
These are visible outside ports. They can be connected to inner ports as proxy.

- Inner components and compositions
User defined architecture. It can be customized via config file before mappd daemon starts or at run time in case with dynamic environments.

- Component 
User defined runnable entity. Contains one or more executable objects (user_code.so) which executes in separate threads. All ports are directly placed as user code main() arguments:

```cpp
int main(int argc, char **argv, std::condition_variable &cv, bool &done, struct cmp_ports_map_t &pm){
	auto sender = pm.get_portbyname("test_sender");
	sender.send(999);
	done = true;
	cv.notify_one();
}
```

- Composition
Contain proxy ports, other compositions or components.

- Ports.
This entity used by components for messaging with world outside. They can be connected or disconnected at runtime with ports of correspond types or same proxies.

