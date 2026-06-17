HPP_REPO=https://github.com/humanoid-path-planner
JRL_REPO=https://github.com/jrl-umi3218

SRC_DIR=${DEVEL_HPP_DIR}/src
INSTALL_HPP_DIR=${DEVEL_HPP_DIR}/install
ROBOTPKG=/opt/openrobots

BUILD_TYPE=Debug
BUILD_FOLDER=build-rel
BUILD_TESTING=OFF
BUILD_JOBS=4

HPP_VERSION=devel
HPP_EXTRA_FLAGS=-DBUILD_TESTING=${BUILD_TESTING} -DAUTO_UNINSTALL=OFF
PYTHON_FLAGS=-DPYTHON_STANDARD_LAYOUT=ON
HPP_CONSTRAINTS_DIR=-Dhpp-constraints_DIR=${INSTALL_HPP_DIR}/lib/cmake/hpp-constraints
HPP_CORE_DIR=-Dhpp-core_DIR=${INSTALL_HPP_DIR}/lib/cmake/hpp-core
HPP_MANIPULATION_DIR=-Dhpp-manipulation_DIR=${INSTALL_HPP_DIR}/lib/cmake/hpp-manipulation
HPP_MANIPULATION_URDF_DIR=-Dhpp-manipulation-urdf_DIR=${INSTALL_HPP_DIR}/lib/cmake/hpp-manipulation-urdf
HPP_SOURCE_INCLUDES=-I${SRC_DIR}/hpp-constraints/include \
	-I${SRC_DIR}/hpp-constraints/${BUILD_FOLDER}/include \
	-I${SRC_DIR}/hpp-core/include \
	-I${SRC_DIR}/hpp-core/${BUILD_FOLDER}/include \
	-I${SRC_DIR}/hpp-manipulation/include \
	-I${SRC_DIR}/hpp-manipulation/${BUILD_FOLDER}/include \
	-I${SRC_DIR}/hpp-manipulation-urdf/include \
	-I${SRC_DIR}/hpp-manipulation-urdf/${BUILD_FOLDER}/include

jrl-cmakemodules_branch=master
jrl-cmakemodules_repository=${JRL_REPO}

hpp-constraints_branch=${HPP_VERSION}
hpp-constraints_repository=${HPP_REPO}
hpp-constraints_extra_flags=${HPP_EXTRA_FLAGS} -DUSE_QPOASES=OFF

hpp-core_branch=${HPP_VERSION}
hpp-core_repository=${HPP_REPO}
hpp-core_extra_flags=${HPP_EXTRA_FLAGS} ${HPP_CONSTRAINTS_DIR}

hpp-manipulation_branch=${HPP_VERSION}
hpp-manipulation_repository=${HPP_REPO}
hpp-manipulation_extra_flags=${HPP_EXTRA_FLAGS} \
	${HPP_CONSTRAINTS_DIR} ${HPP_CORE_DIR}

hpp-manipulation-urdf_branch=${HPP_VERSION}
hpp-manipulation-urdf_repository=${HPP_REPO}
hpp-manipulation-urdf_extra_flags=${HPP_EXTRA_FLAGS} \
	${HPP_CONSTRAINTS_DIR} ${HPP_CORE_DIR} ${HPP_MANIPULATION_DIR}

hpp-python_branch=${HPP_VERSION}
hpp-python_repository=${HPP_REPO}
hpp-python_extra_flags=${HPP_EXTRA_FLAGS} ${PYTHON_FLAGS} \
	${HPP_CONSTRAINTS_DIR} ${HPP_CORE_DIR} ${HPP_MANIPULATION_DIR} \
	${HPP_MANIPULATION_URDF_DIR}

hpp-gepetto-viewer_branch=${HPP_VERSION}
hpp-gepetto-viewer_repository=${HPP_REPO}
hpp-gepetto-viewer_extra_flags=${HPP_EXTRA_FLAGS} ${PYTHON_FLAGS} \
	-DUSE_HPP_PYTHON=ON -DINSTALL_DOCUMENTATION=OFF

# High-level targets

all: hpp-python.install hpp-gepetto-viewer.install
	@echo ""
	@echo "hpp-python + hpp-gepetto-viewer installed. You can now run:"
	@echo "  python3 -c 'from pyhpp.core import Problem; print(\"pyhpp OK\")'"
	@echo "  python3 -c 'from pyhpp_viser import Viewer; print(\"viewer OK\")'"

# Dependencies

jrl-cmakemodules.configure.dep: jrl-cmakemodules.checkout
hpp-constraints.configure.dep: hpp-constraints.checkout jrl-cmakemodules.install
hpp-core.configure.dep: hpp-core.checkout hpp-constraints.install
hpp-manipulation.configure.dep: hpp-manipulation.checkout hpp-core.install
hpp-manipulation-urdf.configure.dep: hpp-manipulation-urdf.checkout hpp-manipulation.install
hpp-python.configure.dep: hpp-python.checkout hpp-constraints.install hpp-manipulation-urdf.install
hpp-gepetto-viewer.configure.dep: hpp-gepetto-viewer.checkout hpp-python.install

# Generic targets

%.checkout:
	if [ -d $(@:.checkout=)/.git ]; then \
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
		-DCMAKE_PREFIX_PATH="${INSTALL_HPP_DIR};${ROBOTPKG};${CMAKE_PREFIX_PATH}" \
		-DCMAKE_INSTALL_LIBDIR=lib \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
		-DCMAKE_CXX_FLAGS="${HPP_SOURCE_INCLUDES} -I${INSTALL_HPP_DIR}/include ${CMAKE_CXX_FLAGS}" \
		-DCMAKE_CXX_FLAGS_DEBUG="-O0 -g0" \
		-DENFORCE_MINIMAL_CXX_STANDARD=ON \
		${$(@:.configure_nodep=)_extra_flags} ..

%.install: %.configure
	${MAKE} -j${BUILD_JOBS} -C ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER} install

%.install_nodep: %.configure_nodep
	${MAKE} -j${BUILD_JOBS} -C ${SRC_DIR}/$(@:.install_nodep=)/${BUILD_FOLDER} install

%.update:
	cd ${SRC_DIR}/$(@:.update=) && git pull origin ${$(@:.update=)_branch} && git submodule update
