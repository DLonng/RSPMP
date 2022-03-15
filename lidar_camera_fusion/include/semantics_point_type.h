#ifndef SEMANTICS_POINT_TYPE
#define SEMANTICS_POINT_TYPE
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

struct PointXYZRGBSemanticsMax {
    PCL_ADD_POINT4D; // Preferred way of adding a XYZ+padding
    PCL_ADD_RGB;

    union // Semantic color
    {
        // 方便进行语义颜色的融合
        struct {
            uint8_t s_b;
            uint8_t s_g;
            uint8_t s_r;
            uint8_t s_a;
        };
        float semantic_color;
    };

    union // Confidences
    {
        float confidence;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned

} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

// here we assume a XYZ + RGB + "sementic_color" + "confidence" (as fields)
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBSemanticsMax,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, semantic_color, semantic_color)(float, confidence, confidence))

struct PointXYZRGBSemanticsBayesian {
    PCL_ADD_POINT4D; // Preferred way of adding a XYZ+padding
    PCL_ADD_RGB;

    union // Semantic colors
    {
        //float data_sem[4];

        struct {
            uint8_t s1_b;
            uint8_t s1_g;
            uint8_t s1_r;
            uint8_t s1_a;

            uint8_t s2_b;
            uint8_t s2_g;
            uint8_t s2_r;
            uint8_t s2_a;

            uint8_t s3_b;
            uint8_t s3_g;
            uint8_t s3_r;
            uint8_t s3_a;

            uint8_t s4_b;
            uint8_t s4_g;
            uint8_t s4_r;
            uint8_t s4_a;
        };

        struct
        {
            float semantic_color1;
            float semantic_color2;
            float semantic_color3;
        };
    };

    union // Confidences
    {
        float data_conf[4];
        struct
        {
            float confidence1;
            float confidence2;
            float confidence3;
        };
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

// here we assume a XYZ + RGB + "sementic_colors" + "confidences" (as fields)
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBSemanticsBayesian,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, semantic_color1, semantic_color1)(float, semantic_color2, semantic_color2)(float, semantic_color3, semantic_color3)(float, confidence1, confidence1)(float, confidence2, confidence2)(float, confidence3, confidence3))


#endif
