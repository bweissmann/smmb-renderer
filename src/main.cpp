#include <QCoreApplication>
#include <QCommandLineParser>

#include <iostream>

#include "pathtracer.h"
#include "scene/scene.h"

#include <QImage>

#include "util/CS123Common.h"

int main(int argc, char *argv[])
{
    unsigned int current_time = time(NULL);
    srand(current_time);
    std::cout << "Random Seed: " << current_time << std::endl;
    QCoreApplication a(argc, argv);

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("scene", "Scene file to be rendered");
    parser.addPositionalArgument("output", "Image file to write the rendered image to");
    parser.addPositionalArgument("section_id", "Index representing the section of the image that should be rendered (default 0)");
    parser.addPositionalArgument("sections", "Total number of sections in the image (default 1)");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(!(args.size() == 2 || args.size() == 4)) {
        std::cerr << "Error: Wrong number of arguments" << std::endl;
        a.exit(1);
        return 1;
    }
    QString scenefile = args[0];
    QString output = args[1];

    /* Set sections based on input arguments or defaults */
    int section_id = 0;
    int sections = 1;
    if (args.size() == 4) {
        bool ok1, ok2;
        section_id = args[2].toInt(&ok1);
        sections = args[3].toInt(&ok2);
        if (!ok1 || !ok2) {
            std::cerr << "Error parsing integer argument" << std::endl;
            a.exit(1);
            return 1;
        }
    }

    int adjusted_image_height = IMAGE_HEIGHT / sections;

    std::cout << "Loaded program with section: " << section_id << " (max " << sections << ")" << std::endl;

//    QImage image(IMAGE_WIDTH, adjusted_image_height, QImage::Format_RGB32);

    Scene *scene;
    if(!Scene::load(scenefile, &scene)) {
        std::cerr << "Error parsing scene file " << scenefile.toStdString() << std::endl;
        a.exit(1);
        return 1;
    }

    PathTracer tracer(IMAGE_WIDTH, IMAGE_HEIGHT, adjusted_image_height, section_id, output);

//    QRgb *data = reinterpret_cast<QRgb *>(image.bits());

    tracer.traceScene(*scene);
    delete scene;

//    bool success = image.save(output);
//    if(!success) {
//        success = image.save(output, "PNG");
//    }
//    if(success) {
//        std::cout << "Wrote rendered image to " << output.toStdString() << std::endl;
//    } else {
//        std::cerr << "Error: failed to write image to " << output.toStdString() << std::endl;
//    }
    a.exit();
}
