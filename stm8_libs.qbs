import qbs
import qbs.FileInfo

StaticLibrary {
    name: "stm8lib"

    Depends { name: "cpp" }
    Depends { name: "common_options" }

    cpp.includePaths: [
        FileInfo.joinPaths(sourceDirectory, "hal"),
        FileInfo.joinPaths(sourceDirectory, "common"),
        FileInfo.joinPaths(sourceDirectory, "drivers"),
        FileInfo.joinPaths(sourceDirectory, "wake"),
    ]

    Group {
        name: "hal"
        prefix: "hal/"
        files: [
            "*.h",
            "*.cpp"
        ]
    }

    Group {
        name: "common"
        prefix: "common/"
        files: [
            "*.h",
            "*.cpp"
        ]
    }

    Group {
        name: "drivers"
        prefix: "drivers/"
        files: [
            "*.h",
            "*.cpp"
        ]
    }

    Group {
        name: "wake"
        prefix: "wake/"
        files: [
            "*.h",
            "*.cpp"
        ]
    }

    Group {
        name: "resources"
        prefix: "resources/"
        files: [
            "*.h",
            "*.cpp"
        ]
    }

    Export {
        Depends { name: "cpp" }
        cpp.includePaths: [
            FileInfo.joinPaths(exportingProduct.sourceDirectory, "hal"),
            FileInfo.joinPaths(exportingProduct.sourceDirectory, "common"),
            FileInfo.joinPaths(exportingProduct.sourceDirectory, "drivers"),
            FileInfo.joinPaths(exportingProduct.sourceDirectory, "wake"),
        ]
    }
}
