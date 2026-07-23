

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>

#include "commonFixture.h"

using namespace ocs2;

class CppAdInterfaceNoParameterFixture : public CommonCppAdNoParameterFixture {};
class CppAdInterfaceParameterizedFixture : public CommonCppAdParameterizedFixture {};

TEST_F(CppAdInterfaceNoParameterFixture, testModelGeneration) {
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, "testModelWithoutParameters");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x).isApprox(testFun(x)));
  ASSERT_TRUE(adInterface.getJacobian(x).isApprox(testJacobian(x)));
  ASSERT_TRUE(adInterface.getHessian(0, x).isApprox(testHessian(x)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x).transpose() * testFun(x)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x).transpose() * testJacobian(x)));
}

TEST_F(CppAdInterfaceParameterizedFixture, testModelGeneration) {
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelWithParameters");

  adInterface.createModels(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x, p).transpose() * testFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x, p).transpose() * testJacobian(x, p)));
}

TEST_F(CppAdInterfaceParameterizedFixture, loadIfAvailable) {
  ocs2::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelLoadIfAvailable");

  adInterface.loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  const auto gnApproximation = adInterface.getGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * testFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(testJacobian(x, p).transpose() * testFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(testJacobian(x, p).transpose() * testJacobian(x, p)));
}

TEST_F(CppAdInterfaceParameterizedFixture, loadIfAvailableRebuildsCorruptedLibrary) {
  const std::string modelName = "testModelLoadIfAvailableCorrupted";
  const boost::filesystem::path tempRoot =
      boost::filesystem::temp_directory_path() / ("ocs2_cppad_" + std::to_string(::getpid()) + "_" + modelName);
  const boost::filesystem::path libraryDir = tempRoot / modelName / "cppad_generated";
  const boost::filesystem::path libraryPath = libraryDir / (modelName + "_lib.so");

  boost::filesystem::remove_all(tempRoot);
  boost::filesystem::create_directories(libraryDir);
  {
    std::ofstream corruptedLibrary(libraryPath.string(), std::ios::binary);
    corruptedLibrary << "bad";
  }

  ocs2::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, modelName, tempRoot.string());
  adInterface.loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::Second, true);

  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.getFunctionValue(x, p).isApprox(testFun(x, p)));
  ASSERT_TRUE(adInterface.getJacobian(x, p).isApprox(testJacobian(x, p)));
  ASSERT_TRUE(adInterface.getHessian(0, x, p).isApprox(testHessian(0, x, p)));
  ASSERT_TRUE(adInterface.getHessian(1, x, p).isApprox(testHessian(1, x, p)));

  ASSERT_TRUE(boost::filesystem::exists(libraryPath));
  ASSERT_GT(boost::filesystem::file_size(libraryPath), 3u);

  boost::filesystem::remove_all(tempRoot);
}
