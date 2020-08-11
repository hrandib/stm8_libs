import qbs
import qbs.FileInfo

StaticLibrary {
    name: "stm8lib"

    Depends { name: "cpp" }
    Depends { name: "Common options" }

    cpp.includePaths: [
        FileInfo.joinPaths(sourceDirectory, "hal"),
    ]

    Group {
        name: "hal"
        prefix: "hal/"
        files: [
            "*.h",
            "*.cpp"
        ]
    }

    Export {
        Depends { name: "cpp" }
        cpp.includePaths: [
            FileInfo.joinPaths(product.sourceDirectory, "hal")
        ]
    }
}
