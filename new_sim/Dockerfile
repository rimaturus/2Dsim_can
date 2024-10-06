# Use the official Ubuntu 22.04 LTS as the base image
FROM ubuntu:22.04

# Set a build argument for the username
ARG USERNAME=wall_e
ARG USER_UID=1000
ARG USER_GID=1000

# Set environment variable to avoid interactive prompts during package installations
ENV DEBIAN_FRONTEND=noninteractive

# Install common utilities, development tools, sudo, and set up locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    nano \
    bash-completion \
    python3-argcomplete \
    lsof \
    vim \
    iputils-ping \
    net-tools \
    locales \
    make \
    gcc \
    g++ \
    wget \
    curl \
    zip \
    unzip \
    tar \
    \
    python3-can \
    can-utils \
    liballegro5-dev \
    libyaml-dev \
    gdb \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Set color support for terminal
ENV TERM=xterm-256color

# Copy configuration files to the container
COPY config/ /site_config/

# Create a non-default user with sudo privileges and add to dialout group
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,dialout $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set the working directory
WORKDIR /home/$USERNAME/sim

# Switch to non-root user for the rest of the build
USER $USERNAME

# Set the default command
CMD [ "bash" ]
