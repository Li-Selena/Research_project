FROM liselena/ros2-jazzy-cuda126:latest

ARG MUJOCO_VERSION=3.12.0
ARG MUJOCO_SHA256=a9367911e6d5eaeade17c2197304687421c1fc932cdf7bcd4cb8cfaf0374dcb2

RUN apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=30 update && \
    apt-get install -y --no-install-recommends \
        libglfw3-dev && \
    archive="/tmp/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" && \
    url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" && \
    for attempt in 1 2 3 4 5 6 7 8 9 10; do \
        curl --retry 3 --retry-delay 2 --retry-all-errors -L -C - -o "${archive}" "${url}" && break; \
    done && \
    echo "${MUJOCO_SHA256}  ${archive}" | sha256sum --check --strict && \
    mkdir -p /opt/mujoco && \
    tar -xzf "${archive}" -C /opt/mujoco --strip-components=1 && \
    rm -f "${archive}" && \
    printf '%s\n' '/opt/mujoco/lib' > /etc/ld.so.conf.d/mujoco.conf && \
    ldconfig && \
    rm -rf /var/lib/apt/lists/*

ENV MUJOCO_DIR=/opt/mujoco \
    CMAKE_PREFIX_PATH=/opt/mujoco
