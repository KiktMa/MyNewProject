#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>


using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;

/*
*  @brief �˲�����Ⱥ���޳�����
*  param[in] cloud:�������  output:�˲���ĵ���
*/
void octreeRemovePoints(pcXYZIPtr cloud, pcXYZIPtr output)
{
	pcl::octree::OctreePointCloud<pcl::PointXYZI> octree(0.1);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();   // ����Octree
	vector<int> vec_point_index, vec_total_index;    //  �����ڵ��������Ҫɾ���ĵ������
	vector<pcl::octree::OctreeKey> vec_key;
	for (auto iter = octree.leaf_begin(); iter != octree.leaf_end(); ++iter)
	{
		auto key = iter.getCurrentOctreeKey();
		vec_key.emplace_back(key);
		auto it_key = octree.findLeaf(key.x, key.y, key.z);
		if (it_key != nullptr)
		{
			vec_point_index = iter.getLeafContainer().getPointIndicesVector();
			if (vec_point_index.size() < 10)             // �����ڵ�С��10ʱɾ��
			{
				for (size_t i = 0; i < vec_point_index.size(); i++)
				{
					vec_total_index.push_back(vec_point_index[i]);
				}
			}
		}
	}
	// ʹ��pcl index �˲�
	pcl::PointIndices::Ptr outliners(new pcl::PointIndices());
	outliners->indices.resize(vec_total_index.size());
	for (size_t i = 0; i < vec_total_index.size(); i++)
	{
		outliners->indices[i] = vec_total_index[i];
	}
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(outliners);
	extract.setNegative(true);
	extract.filter(*output);
}


int main(int argc, char** argv)
{
	// ����������ƶ���
	// pcXYZIPtr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
	// ���ļ���ȡ��������
	// pcl::io::loadPCDFile<pcl::PointXYZI>("D:\\JavaConsist\\MapData\\180m_pointcloud\\corrected-LJYY-Cloud-1-0-9 - Cloud-all.pcd", *inputCloud);

	// ����������ƶ���
	// pcXYZIPtr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);

	// ������Ⱥ���޳�����
	// octreeRemovePoints(inputCloud, filteredCloud);

	// ���˲���ĵ��Ʊ��浽�ļ�
	// pcl::io::savePCDFile<pcl::PointXYZI>("filtered_cloud.pcd", *filteredCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    //reader.read("D:\\JavaConsist\\MapData\\180m_pointcloud\\corrected-LJYY-Cloud-1-0-9 - Cloud-clean.pcd", *cloud);
    reader.read("C:\\Users\\mj\\Desktop\\corrected-LJYY-Cloud-1-0-9 - Cloud - Cloud.pcd", *cloud);

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
	
    pcl::PointXYZ min_point, max_point;
    feature_extractor.getAABB(min_point, max_point);
	
    // ������С��������������С��Χ�еİ˸�����
    pcl::PointXYZ vertex1(min_point.x, min_point.y, min_point.z);
    pcl::PointXYZ vertex2(max_point.x, min_point.y, min_point.z);
    pcl::PointXYZ vertex3(min_point.x, max_point.y, min_point.z);
    pcl::PointXYZ vertex4(max_point.x, max_point.y, min_point.z);
    pcl::PointXYZ vertex5(min_point.x, min_point.y, max_point.z);
    pcl::PointXYZ vertex6(max_point.x, min_point.y, max_point.z);
    pcl::PointXYZ vertex7(min_point.x, max_point.y, max_point.z);
    pcl::PointXYZ vertex8(max_point.x, max_point.y, max_point.z);
	
    // ����˸������������Ϣ
    std::cout << "Vertex 1: (" << vertex1.x << ", " << vertex1.y << ", " << vertex1.z << ")" << std::endl;
    std::cout << "Vertex 2: (" << vertex2.x << ", " << vertex2.y << ", " << vertex2.z << ")" << std::endl;
    std::cout << "Vertex 3: (" << vertex3.x << ", " << vertex3.y << ", " << vertex3.z << ")" << std::endl;
    std::cout << "Vertex 4: (" << vertex4.x << ", " << vertex4.y << ", " << vertex4.z << ")" << std::endl;
    std::cout << "Vertex 5: (" << vertex5.x << ", " << vertex5.y << ", " << vertex5.z << ")" << std::endl;
    std::cout << "Vertex 6: (" << vertex6.x << ", " << vertex6.y << ", " << vertex6.z << ")" << std::endl;
    std::cout << "Vertex 7: (" << vertex7.x << ", " << vertex7.y << ", " << vertex7.z << ")" << std::endl;
    std::cout << "Vertex 8: (" << vertex8.x << ", " << vertex8.y << ", " << vertex8.z << ")" << std::endl;
	
	
    std::cout << "Minimum point: (" << min_point.x << ", " << min_point.y << ", " << min_point.z << ")" << std::endl;
    std::cout << "Maximum point: (" << max_point.x << ", " << max_point.y << ", " << max_point.z << ")" << std::endl;

    return 0;
}