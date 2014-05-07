#############################################################################
# Makefile for building: raymini
# Generated by qmake (2.01a) (Qt 4.8.4) on: ?? 5? 6 21:44:45 2014
# Project:  raymini.pro
# Template: app
# Command: /usr/bin/qmake-qt4 -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile raymini.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DQT_NO_DEBUG -DQT_XML_LIB -DQT_OPENGL_LIB -DQT_GUI_LIB -DQT_CORE_LIB -DQT_SHARED
CFLAGS        = -m64 -pipe -O2 -D_REENTRANT -Wall -W $(DEFINES)
CXXFLAGS      = -m64 -pipe -O2 -D_REENTRANT -Wall -W $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I. -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtGui -I/usr/include/qt4/QtOpenGL -I/usr/include/qt4/QtXml -I/usr/include/qt4 -I/usr/X11R6/include -Itmp
LINK          = g++
LFLAGS        = -m64 -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/X11R6/lib64 -L/usr/lib/x86_64-linux-gnu -lGLEW -lqglviewer-qt4 -lpthread -lGL -lQtXml -lQtOpenGL -lQtGui -lQtCore 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/bin/qmake-qt4
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = tmp/

####### Files

SOURCES       = Window.cpp \
		GLViewer.cpp \
		QTUtils.cpp \
		Vertex.cpp \
		Triangle.cpp \
		Mesh.cpp \
		BoundingBox.cpp \
		Material.cpp \
		Object.cpp \
		Light.cpp \
		Scene.cpp \
		RayTracer.cpp \
		Ray.cpp \
		KDTree.cpp \
		Main.cpp tmp/moc_Window.cpp \
		tmp/moc_GLViewer.cpp \
		tmp/moc_QTUtils.cpp
OBJECTS       = tmp/Window.o \
		tmp/GLViewer.o \
		tmp/QTUtils.o \
		tmp/Vertex.o \
		tmp/Triangle.o \
		tmp/Mesh.o \
		tmp/BoundingBox.o \
		tmp/Material.o \
		tmp/Object.o \
		tmp/Light.o \
		tmp/Scene.o \
		tmp/RayTracer.o \
		tmp/Ray.o \
		tmp/KDTree.o \
		tmp/Main.o \
		tmp/moc_Window.o \
		tmp/moc_GLViewer.o \
		tmp/moc_QTUtils.o
DIST          = /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_phonon.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/unix/opengl.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		raymini.pro
QMAKE_TARGET  = raymini
DESTDIR       = 
TARGET        = raymini

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: raymini.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/modules/qt_phonon.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/unix/opengl.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		/usr/lib/x86_64-linux-gnu/libqglviewer-qt4.prl \
		/usr/lib/x86_64-linux-gnu/libQtXml.prl \
		/usr/lib/x86_64-linux-gnu/libQtOpenGL.prl \
		/usr/lib/x86_64-linux-gnu/libQtGui.prl \
		/usr/lib/x86_64-linux-gnu/libQtCore.prl
	$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile raymini.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/modules/qt_phonon.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/unix/opengl.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
/usr/lib/x86_64-linux-gnu/libqglviewer-qt4.prl:
/usr/lib/x86_64-linux-gnu/libQtXml.prl:
/usr/lib/x86_64-linux-gnu/libQtOpenGL.prl:
/usr/lib/x86_64-linux-gnu/libQtGui.prl:
/usr/lib/x86_64-linux-gnu/libQtCore.prl:
qmake:  FORCE
	@$(QMAKE) -spec /usr/share/qt4/mkspecs/linux-g++-64 -o Makefile raymini.pro

dist: 
	@$(CHK_DIR_EXISTS) tmp/raymini1.0.0 || $(MKDIR) tmp/raymini1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) tmp/raymini1.0.0/ && $(COPY_FILE) --parents Window.h GLViewer.h QTUtils.h Vertex.h Triangle.h Mesh.h BoundingBox.h Material.h Object.h Light.h Scene.h RayTracer.h Ray.h KDTree.h tmp/raymini1.0.0/ && $(COPY_FILE) --parents Window.cpp GLViewer.cpp QTUtils.cpp Vertex.cpp Triangle.cpp Mesh.cpp BoundingBox.cpp Material.cpp Object.cpp Light.cpp Scene.cpp RayTracer.cpp Ray.cpp KDTree.cpp Main.cpp tmp/raymini1.0.0/ && (cd `dirname tmp/raymini1.0.0` && $(TAR) raymini1.0.0.tar raymini1.0.0 && $(COMPRESS) raymini1.0.0.tar) && $(MOVE) `dirname tmp/raymini1.0.0`/raymini1.0.0.tar.gz . && $(DEL_FILE) -r tmp/raymini1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all: tmp/moc_Window.cpp tmp/moc_GLViewer.cpp tmp/moc_QTUtils.cpp
compiler_moc_header_clean:
	-$(DEL_FILE) tmp/moc_Window.cpp tmp/moc_GLViewer.cpp tmp/moc_QTUtils.cpp
tmp/moc_Window.cpp: GLViewer.h \
		Scene.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Light.h \
		QTUtils.h \
		Window.h
	/usr/lib/x86_64-linux-gnu/qt4/bin/moc $(DEFINES) $(INCPATH) Window.h -o tmp/moc_Window.cpp

tmp/moc_GLViewer.cpp: Scene.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Light.h \
		GLViewer.h
	/usr/lib/x86_64-linux-gnu/qt4/bin/moc $(DEFINES) $(INCPATH) GLViewer.h -o tmp/moc_GLViewer.cpp

tmp/moc_QTUtils.cpp: QTUtils.h
	/usr/lib/x86_64-linux-gnu/qt4/bin/moc $(DEFINES) $(INCPATH) QTUtils.h -o tmp/moc_QTUtils.cpp

compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: compiler_moc_header_clean 

####### Compile

tmp/Window.o: Window.cpp Window.h \
		GLViewer.h \
		Scene.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Light.h \
		QTUtils.h \
		RayTracer.h \
		SpaceMgr.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Window.o Window.cpp

tmp/GLViewer.o: GLViewer.cpp GLViewer.h \
		Scene.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Light.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/GLViewer.o GLViewer.cpp

tmp/QTUtils.o: QTUtils.cpp QTUtils.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/QTUtils.o QTUtils.cpp

tmp/Vertex.o: Vertex.cpp Vertex.h \
		Vec3D.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Vertex.o Vertex.cpp

tmp/Triangle.o: Triangle.cpp Triangle.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Triangle.o Triangle.cpp

tmp/Mesh.o: Mesh.cpp Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Mesh.o Mesh.cpp

tmp/BoundingBox.o: BoundingBox.cpp BoundingBox.h \
		Vec3D.h \
		Triangle.h \
		Mesh.h \
		Vertex.h \
		Edge.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/BoundingBox.o BoundingBox.cpp

tmp/Material.o: Material.cpp Material.h \
		Vec3D.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Material.o Material.cpp

tmp/Object.o: Object.cpp Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Object.o Object.cpp

tmp/Light.o: Light.cpp Light.h \
		Vec3D.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Light.o Light.cpp

tmp/Scene.o: Scene.cpp Scene.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Light.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Scene.o Scene.cpp

tmp/RayTracer.o: RayTracer.cpp RayTracer.h \
		Vec3D.h \
		Light.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Scene.h \
		SpaceMgr.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/RayTracer.o RayTracer.cpp

tmp/Ray.o: Ray.cpp Ray.h \
		Vec3D.h \
		BoundingBox.h \
		Triangle.h \
		Mesh.h \
		Vertex.h \
		Edge.h \
		Object.h \
		Material.h \
		KDTree.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Ray.o Ray.cpp

tmp/KDTree.o: KDTree.cpp KDTree.h \
		BoundingBox.h \
		Vec3D.h \
		Triangle.h \
		Mesh.h \
		Vertex.h \
		Edge.h \
		Ray.h \
		Object.h \
		Material.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/KDTree.o KDTree.cpp

tmp/Main.o: Main.cpp Window.h \
		GLViewer.h \
		Scene.h \
		Object.h \
		Mesh.h \
		Vertex.h \
		Vec3D.h \
		Triangle.h \
		Edge.h \
		Material.h \
		Ray.h \
		BoundingBox.h \
		KDTree.h \
		Light.h \
		QTUtils.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/Main.o Main.cpp

tmp/moc_Window.o: tmp/moc_Window.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/moc_Window.o tmp/moc_Window.cpp

tmp/moc_GLViewer.o: tmp/moc_GLViewer.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/moc_GLViewer.o tmp/moc_GLViewer.cpp

tmp/moc_QTUtils.o: tmp/moc_QTUtils.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o tmp/moc_QTUtils.o tmp/moc_QTUtils.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

