FROM docker.io/zmkfirmware/zmk-dev-arm:stable

WORKDIR /workspace

# Cache west init + ZMK/Zephyr fetch in a layer
COPY config/west.yml config/west.yml
RUN west init -l config && west update --narrow --fetch-opt=--depth=1

COPY . .

ENTRYPOINT ["/bin/bash"]
