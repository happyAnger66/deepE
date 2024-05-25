# Copyright (C) @2024 zhangxiaoan . All rights reserved.
# Author: zhangxiaoan
# Contact: happyAnger66@163.com


from setuptools import setup, find_packages

package_name = 'deepE'

setup(name='deepE',
      version='1.0',
      description='deep edge perf tools',
      author='zhangxiaoan',
      author_email='zhangxiaoan_v@didiglobal.com',
      url='https://github.com/happyAnger66/deepE',
      packages=find_packages(),
      entry_points={
            'console_scripts': [
                  'deepE=perf_processor.data_pb:main'
            ]
      }
)