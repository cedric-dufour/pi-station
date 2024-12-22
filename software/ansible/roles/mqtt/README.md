Ansible role `mqtt` - MQTT setup
=====

This role installs and pre-configures the various MQTT components (applications)
via [Docker Compose][docker-compose].

[docker-compose]: https://docs.docker.com/compose/

In order for Ansible to manage those Docker Compose'd applications, one must add
the corresponding `docker` role stanza:

``` yaml
DOCKER_ENABLE: true
DOCKER_NETWORKS:
    "mqtt":
    ipam_config:
        - subnet: "192.168.183.0/24"
          gateway: "192.168.183.1"
DOCKER_COMPOSE_APPS:
  "mqtt":
    manage: false  # set to true after Ansible has also passed the `mqtt` role
    project_src: "/opt/mqtt"
    environment:
      COMPOSE_PROFILES: "mosquitto,zigbee2mqtt"
      ZIGBEE2MQTT_DEVICE: "/dev/serial/by-id/..."
      # (see the docker-compose.yaml file for other environment variables)
```
