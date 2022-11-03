# For better backword compatibility...
FROM ubuntu:16.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update
# install pyenv deps
RUN apt-get install -y make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl git sudo

RUN \
  useradd user && \
  echo "user ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/user && \
  chmod 0440 /etc/sudoers.d/user && \
  mkdir -p /home/user && \
  chown user:user /home/user && \
  chsh -s /bin/bash user

WORKDIR /home/user
USER user
CMD /bin/bash

ENV HOME /home/user
ENV PYENV_ROOT $HOME/.pyenv
ENV PATH $PYENV_ROOT/shims:$PYENV_ROOT/bin:$PATH
RUN \
  curl https://pyenv.run|bash && \
  echo 'eval "$(pyenv init -)"' >> ~/.bashrc
COPY python_versions.txt $HOME/
RUN  cat python_versions.txt | while read version; do pyenv install $version ; done

# update cmake to 3.20
RUN sudo apt-get remove cmake
RUN wget -nv https://cmake.org/files/v3.20/cmake-3.20.0-rc1-linux-x86_64.tar.gz
RUN tar -xf cmake-3.20.0-rc1-linux-x86_64.tar.gz 
RUN sudo ln -sf ~/cmake-3.20.0-rc1-linux-x86_64/bin/* /usr/bin

# Prebuild 
RUN sudo apt-get install -y libeigen3-dev libgtest-dev libgflags-dev libgoogle-glog-dev libprotobuf-dev protobuf-compiler
#COPY --chown=user .ssh /home/user/.ssh
#RUN git clone git@github.com:HiroIshida/voxbloxpy.git
RUN git clone https://github.com/HiroIshida/voxbloxpy.git
RUN cat python_versions.txt | while read version; do pyenv global $version && pip install scikit-build; done
RUN cd voxbloxpy && git submodule update --init 

# ad hoc src install gflags (this extra step required because build on legacy ubuntu 16.04)
RUN \
    git clone https://github.com/gflags/gflags.git && \
    cd gflags && mkdir build && \
    cd build && cmake .. && make -j2 && sudo make install

RUN \
    cd voxbloxpy && \
    cat ../python_versions.txt | while read version; do pyenv global $version && python setup.py bdist_wheel -p manylinux1_x86_64; done

COPY check_glibcxx.sh $HOME/
RUN bash ./check_glibcxx.sh

RUN pyenv global $(tail -n1 python_versions.txt)
RUN pip install twine
