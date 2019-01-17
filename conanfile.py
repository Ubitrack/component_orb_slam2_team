from conans import ConanFile, CMake, tools


class UbitrackOrbSlam2TeamConan(ConanFile):
    name = "ubitrack_component_orb_slam2_team"
    version = "1.3.0"

    description = "Ubitrack ORB_SLAM2_TEAM Component"
    url = "https://github.com/Ubitrack/component_orb_slam2_team.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    requires = (
        "ubitrack_core/%s@ubitrack/stable" % version,
        "ubitrack_vision/%s@ubitrack/stable" % version,
        "ubitrack_dataflow/%s@ubitrack/stable" % version,
        "ubitrack_component_core/%s@ubitrack/stable" % version,
       )

    default_options = (
        "ubitrack_core:shared=True",
        "ubitrack_vision:shared=True",
        "ubitrack_dataflow:shared=True",
        )

    # all sources are deployed with the package
    exports_sources = "doc/*", "src/*", "CMakeLists.txt"

    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
        self.copy(pattern="*.dylib*", dst="lib", src="lib") 
        self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        #cmake.definitions['HAVE_GLAD'] = self.options['ubitrack_vision'].opengl_extension_wrapper == 'glad'
        #cmake.definitions['HAVE_GLEW'] = self.options['ubitrack_vision'].opengl_extension_wrapper == 'glew'
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        pass

    def package_info(self):
        pass

    def package_id(self):
        self.info.requires["ubitrack_vision"].full_package_mode()
