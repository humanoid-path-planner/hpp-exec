HPP_REPO=https://github.com/humanoid-path-planner
JRL_REPO=https://github.com/jrl-umi3218

SRC_DIR=${DEVEL_HPP_DIR}/src
INSTALL_HPP_DIR=${DEVEL_HPP_DIR}/install
ROBOTPKG=/opt/openrobots

BUILD_TYPE=Release
BUILD_FOLDER=build-rel
BUILD_TESTING=OFF

HPP_EXTRA_FLAGS=-DBUILD_TESTING=${BUILD_TESTING}
PYTHON_FLAGS=-DPYTHON_STANDARD_LAYOUT=ON

jrl-cmakemodules_branch=master
jrl-cmakemodules_repository=${JRL_REPO}

# hpp-manipulation from source (devel) to get latest API (e.g. approachingDirection)
hpp-manipulation_branch=devel
hpp-manipulation_repository=${HPP_REPO}
hpp-manipulation_extra_flags=${HPP_EXTRA_FLAGS}

hpp-python_branch=devel
hpp-python_repository=${HPP_REPO}
hpp-python_extra_flags=${HPP_EXTRA_FLAGS} ${PYTHON_FLAGS}

# High-level targets

all: hpp-python.install
	@echo ""
	@echo "hpp-python installed. You can now run:"
	@echo "  python3 -c 'from pyhpp.core import Problem; print(\"pyhpp OK\")'"

# Dependencies

jrl-cmakemodules.configure.dep: jrl-cmakemodules.checkout
hpp-manipulation.configure.dep: hpp-manipulation.checkout jrl-cmakemodules.install
hpp-python.configure.dep: hpp-python.checkout jrl-cmakemodules.install hpp-manipulation.install

# Generic targets

%.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checked out."; \
	else \
		git clone --recursive -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
	fi

%.configure.dep: %.checkout

%.configure: %.configure.dep
	${MAKE} $(@:.configure=).configure_nodep

%.configure_nodep: %.checkout
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} \
		-DCMAKE_PREFIX_PATH="${INSTALL_HPP_DIR};${ROBOTPKG}" \
		-DCMAKE_INSTALL_LIBDIR=lib \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
		-DENFORCE_MINIMAL_CXX_STANDARD=ON \
		${$(@:.configure_nodep=)_extra_flags} ..

%.install: %.configure
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER} install

%.install_nodep: %.configure_nodep
	${MAKE} -C ${SRC_DIR}/$(@:.install_nodep=)/${BUILD_FOLDER} install

%.update:
	cd ${SRC_DIR}/$(@:.update=) && git pull origin ${$(@:.update=)_branch} && git submodule update
