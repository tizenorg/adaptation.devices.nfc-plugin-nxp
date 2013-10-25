Name:       nfc-plugin-nxp
Summary:    NFC Plugin for NXP Solution
Version:    0.0.8
Release:    0
Group:      TO_BE/FILLED_IN
License:    TO BE FILLED IN
Source0:    %{name}-%{version}.tar.gz
BuildRequires: cmake
BuildRequires: pkgconfig(glib-2.0)
BuildRequires: pkgconfig(vconf)
BuildRequires: pkgconfig(dlog)
BuildRequires: pkgconfig(nfc-common-lib)

%description
Description: NFC Plugin for NXP Solution


%prep
%setup -q

%build
cmake . -DCMAKE_INSTALL_PREFIX=%{_prefix}

make %{?jobs:-j%jobs}


%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/usr/share/license
cp -af LICENSE.APLv2 %{buildroot}/usr/share/license/%{name}

%make_install

%post
# file owner
if [ "${USER}" = "root" ]
then
	vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -g 6517 -i -f
else
	vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -f
fi


%files
%manifest nfc-plugin-nxp.manifest
%defattr(-,root,root,-)
/usr/lib/libnfc-plugin.so
/usr/share/license/%{name}
