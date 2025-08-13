
# This script just creates compressed files of OpenMesh sources

# Extract Version Information
VERSION=OpenMesh-$(cat CMakeLists.txt | grep VERSION | grep -v cmake | grep -v CMAKE | tr -d "VERSION=" | tr -d " ")

# Create Publishing directory
mkdir $VERSION

# Move all files into Publishing directory
mv CHANGELOG.md   $VERSION/
mv cmake          $VERSION/
mv CMakeLists.txt $VERSION/
mv debian         $VERSION/
mv Doc            $VERSION/
mv LICENSE        $VERSION/
mv README.md      $VERSION/
mv src            $VERSION/
mv cmake-library  $VERSION/ 
mv openmesh.pc.in $VERSION/

tar cjf $VERSION.tar.bz2 $VERSION 
tar czf $VERSION.tar.gz $VERSION 
zip -9 -q -r $VERSION.zip $VERSION 

