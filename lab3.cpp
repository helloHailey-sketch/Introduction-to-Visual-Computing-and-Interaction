#include "Labs/3-Rendering/tasks.h"
#include <algorithm>

namespace VCX::Labs::Rendering {

    glm::vec4 GetTexture(Engine::Texture2D<Engine::Formats::RGBA8> const & texture, glm::vec2 const & uvCoord) {
        if (texture.GetSizeX() == 1 || texture.GetSizeY() == 1) return texture.At(0, 0);
        glm::vec2 uv      = glm::fract(uvCoord);
        uv.x              = uv.x * texture.GetSizeX() - .5f;
        uv.y              = uv.y * texture.GetSizeY() - .5f;
        std::size_t xmin  = std::size_t(glm::floor(uv.x) + texture.GetSizeX()) % texture.GetSizeX();
        std::size_t ymin  = std::size_t(glm::floor(uv.y) + texture.GetSizeY()) % texture.GetSizeY();
        std::size_t xmax  = (xmin + 1) % texture.GetSizeX();
        std::size_t ymax  = (ymin + 1) % texture.GetSizeY();
        float       xfrac = glm::fract(uv.x), yfrac = glm::fract(uv.y);
        return glm::mix(glm::mix(texture.At(xmin, ymin), texture.At(xmin, ymax), yfrac), glm::mix(texture.At(xmax, ymin), texture.At(xmax, ymax), yfrac), xfrac);
    }

    glm::vec4 GetAlbedo(Engine::Material const & material, glm::vec2 const & uvCoord) {
        glm::vec4 albedo       = GetTexture(material.Albedo, uvCoord);
        glm::vec3 diffuseColor = albedo;
        return glm::vec4(glm::pow(diffuseColor, glm::vec3(2.2)), albedo.w);
    }

    /******************* 1. Ray-triangle intersection *****************/
    bool IntersectTriangle(Intersection & output, Ray const & ray, glm::vec3 const & p1, glm::vec3 const & p2, glm::vec3 const & p3) {
        // your code here
        //求光线 ray 与三角形 (p1, p2, p3) 之间的交点，如果交点存在，在 output 中设置交点与光线原点的距离 t 以及交点在三角形中的质心坐标 (u, v) 并返回 true ；否则返回 false 
        glm::vec3 E1 = p2 - p1;
        glm::vec3 E2 = p3 - p1;
        glm::vec3 T = ray.Origin - p1;
        auto D = ray.Direction;
        glm::vec3 P = glm::cross(D, E2);
        glm::vec3 Q = glm::cross(T, E1);
        float t = glm::dot(Q, E2)/glm::dot(P, E1);
        float u = glm::dot(P, T)/glm::dot(P, E1);
        float v = glm::dot(Q, D)/glm::dot(P, E1);
        double eps = -1e-6;
        if(t > eps && u > eps && v > eps && 1-u-v > eps){
            output.t = t;
            output.u = u;
            output.v = v;
            return true;
        }

        return false;
    }

    glm::vec3 RayTrace(const RayIntersector & intersector, Ray ray, int maxDepth, bool enableShadow) {
        glm::vec3 color(0.0f); //光线积累的颜色的初始值
        glm::vec3 weight(1.0f); 

        for (int depth = 0; depth < maxDepth; depth++) {
            auto rayHit = intersector.IntersectRay(ray);
            if (! rayHit.IntersectState) return color;
            const glm::vec3 pos       = rayHit.IntersectPosition;　// 相交点的位置
            const glm::vec3 n         = rayHit.IntersectNormal;　// 相交点的法线向量
            const glm::vec3 kd        = rayHit.IntersectAlbedo;// 漫反射颜色
            const glm::vec3 ks        = rayHit.IntersectMetaSpec;// 镜面反射颜色
            const float     alpha     = rayHit.IntersectAlbedo.w;// 透明度（决定反射/折射比例）
            const float     shininess = rayHit.IntersectMetaSpec.w * 256;// 镜面光高光指数

/*           if (alpha < .2)
                ray.Origin = pos;
            while (alpha < .2)
            {
                auto rayHit = intersector.IntersectRay(ray);
                if (! rayHit.IntersectState) return color;
                const glm::vec3 pos       = rayHit.IntersectPosition;
                const glm::vec3 n         = rayHit.IntersectNormal;
                const glm::vec3 kd        = rayHit.IntersectAlbedo;
                const glm::vec3 ks        = rayHit.IntersectMetaSpec;
                const float alpha     = rayHit.IntersectAlbedo.w;
                const float shininess = rayHit.IntersectMetaSpec.w * 256;
                if (alpha < .2)
                    ray.Origin = pos;
            } 
*/ 
            glm::vec3 result(0.0f);
            /******************* 2. Whitted-style ray tracing *****************/
            // your code here
            result += kd * intersector.InternalScene->AmbientIntensity;
            
            for (const Engine::Light & light : intersector.InternalScene->Lights) {
                glm::vec3 l; //光线方向
                float     attenuation; //衰减因子，光源强度随距离减弱
                /******************* 3. Shadow ray *****************/
                if (light.Type == Engine::LightType::Point) {
                    l           = light.Position - pos;
                    attenuation = 1.0f / glm::dot(l, l); //点光
                    if (enableShadow) {
                        // your code here
                        Ray shadowRay(pos, l);
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        const float alpha = shadowHit.IntersectAlbedo.w; //透明度 <0.2视为透明
/*                        while (shadowHit.IntersectState && alpha < 0.2)
                            shadowHit = intersector.IntersectRay(Ray(shadowHit.IntersectPosition, glm::normalize(l)));
*/                        float distance = glm::length(shadowHit.IntersectPosition - pos); //交点距离
                        if (shadowHit.IntersectState && distance < glm::length(light.Position - pos) && alpha >= 0.2) {
                            attenuation = 0.0f;
                            continue; // 被遮挡，跳过当前光源
                        }
                    }
                } else if (light.Type == Engine::LightType::Directional) {
                    l           = light.Direction;
                    attenuation = 1.0f; //有向光
                    if (enableShadow) {
                        // your code here
                        Ray shadowRay(pos, l);
                        auto shadowHit = intersector.IntersectRay(shadowRay);
                        const float alpha = shadowHit.IntersectAlbedo.w; //透明度 <0.2视为透明
/*                        while (shadowHit.IntersectState && alpha < 0.2)
                            shadowHit = intersector.IntersectRay(Ray(shadowHit.IntersectPosition, glm::normalize(l)));
*/                        float distance = glm::length(shadowHit.IntersectPosition - pos);//交点距离
                        if (shadowHit.IntersectState && distance < glm::length(light.Position - pos) && alpha >= 0.2) {
                            attenuation = 0.0f;
                            continue; // 被遮挡，跳过当前光源
                        }
                    }
                }

                /******************* 2. Whitted-style ray tracing *****************/
                // your code here
                //if ! in_shadow ( shadow_ray , light) radiance += phong_illumination (point , ray , light)
 /*               glm::vec3 r = 2.0f * n * glm::dot(n, l) - l; //reflected ray
                float cos_phi = glm::dot(r, ray.Direction) / (glm::length(r) * glm::length(ray.Direction)); //反射光和观察点的夹角
                glm::vec3 s = ks * glm::pow(cos_phi, shininess);
                float cos_theta = glm::dot(n, l) / (glm::length(n) * glm::length(l)); //太阳光和法线的夹角
                glm::vec3 d = kd * glm::max(0.0f, cos_theta);
                result += light.Intensity * attenuation * (s + d);
*/                 
                glm::vec3 h  = glm::normalize(-ray.Direction + glm::normalize(l));
                float spec = glm::pow(glm::max(glm::dot(h, n), 0.0f), shininess);
                float diff = glm::max(glm::dot(glm::normalize(l), n), 0.0f);
                result += light.Intensity * attenuation * (diff * kd + spec * ks);
                
            }

            if (alpha < 0.9) {
                // refraction折射
                // accumulate color
                glm::vec3 R = alpha * glm::vec3(1.0f);
                color += weight * R * result;
                weight *= glm::vec3(1.0f) - R;

                // generate new ray
                ray = Ray(pos, ray.Direction);
            } else {
                // reflection
                // accumulate color
                glm::vec3 R = ks * glm::vec3(0.5f);
                color += weight * (glm::vec3(1.0f) - R) * result;
                weight *= R;

                // generate new ray
                glm::vec3 out_dir = ray.Direction - glm::vec3(2.0f) * n * glm::dot(n, ray.Direction);
                ray               = Ray(pos, out_dir);
            }
        }

        return color;
    }
} // namespace VCX::Labs::Rendering