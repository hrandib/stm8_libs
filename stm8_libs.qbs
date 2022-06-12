import qbs
import qbs.FileInfo

StaticLibrary {
    name: "stm8lib"

    Depends { name: "cpp" }
    Depends { name: "common_options" }

    cpp.includePaths: [
        FileInfo.joinPaths(sourceDirectory, "hal"),
        FileInfo.joinPaths(sourceDirectory, "common")
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

    Export {
        Depends { name: "cpp" }
        cpp.includePaths: [
            FileInfo.joinPaths(exportingProduct.sourceDirectory, "hal"),
            FileInfo.joinPaths(exportingProduct.sourceDirectory, "common")
        ]
    }
}
