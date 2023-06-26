FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

RUN apt update -y
RUN apt install sudo -y

# Install needed packages
RUN sudo apt install -y python3 python3-pip
RUN sudo apt install -y curl wget
RUN sudo apt install -y zip unzip

# Install sdkman
RUN curl -s "https://get.sdkman.io?rcupdate=true" | bash



# Install java
RUN bash -c "source /root/.sdkman/bin/sdkman-init.sh && sdk install java 17.0.7-amzn"

# # Install gradle
RUN bash -c "source /root/.sdkman/bin/sdkman-init.sh && sdk install gradle"

RUN echo "source /root/.sdkman/bin/sdkman-init.sh" >> /root/.bashrc

# Install WPILib
RUN wget https://github.com/wpilibsuite/allwpilib/releases/download/v2023.4.3/WPILib_Linux-2023.4.3.tar.gz -O wpilib.tar.gz
RUN mkdir -p /root/wpilib/2023
RUN tar -zxvf wpilib.tar.gz -C /root/wpilib/2023